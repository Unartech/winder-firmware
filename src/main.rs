#![no_std]
#![no_main]

/*
    Author: Rajesh Pachaikani
    Date: 09/05/2022
    Title: Firmware for filament extrusion line control and monitoring
    Description:
        Pin Configuration:
            - PA6: Winder Motor Pulse ( Connected to Timer 3 Channel 1)
            - PB12: Winder Motor Direction
            - PA1: Linear Guide Motor Pulse ( Connected to Timer 2 Channel 2)
            - PA15: Linear Guide Motor Direction
            - PA2: Push Button to reverse the direction of Winder Motor
            - PA3: Push Button for Spool Change
            - PB0: Push Button for Fast Forward Winder Motor
            - PB4: Proximity Sensor
            - PB6: I2C SCL
            - PB7: I2C SDA
            - PA11: USB -ve Line
            - PA12: USB +ve Line
            - PA1: Winder motor speed connected to ADC1_IN1
            - PA2: Linear guide motor speed connected to ADC1_IN2
*/


use core::f32::consts::PI;
use rtic::app;
use panic_semihosting as _;

const ROLLER_DIAMETER: f32 = 0.12; // in meters

fn pulse_count_to_length(pulse_count: u32) -> f32 {
    // Pulse count to length conversion
        let circumference = 2.0 * PI * ROLLER_DIAMETER;
    // Two pulses per revolution
        circumference * (pulse_count as f32 / 2.0)
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {

    use core::fmt::Write;
    use core::sync::atomic::{
        AtomicUsize,
        Ordering,
    };
    use heapless::{
        String,
    };
    use lcd_lcm1602_i2c::Lcd;
    use stm32f1xx_hal::{
        stm32,
        prelude::*,
        i2c::{BlockingI2c, DutyCycle, Mode},
    };
    use stm32f1xx_hal::gpio::{Alternate, CRL, Edge, ExtiPin, Input, OpenDrain, Output, Pin, PullUp, PushPull};
    use stm32f1xx_hal::pac::{I2C1, TIM4};
    use stm32f1xx_hal::gpio::CRH;
    use stm32f1xx_hal::timer::Delay;
    use crate::pulse_count_to_length;


    static PULSE_COUNT: AtomicUsize = AtomicUsize::new(0);

    #[shared]
    struct Shared {

    }

    #[local]
    struct Local {
        // Declaring Lcd as local variable to be used in the idle task
        i2c: BlockingI2c<
            I2C1,
            (
                Pin<Alternate<OpenDrain>, stm32f1xx_hal::gpio::CRL, 'B', 6>,
                Pin<Alternate<OpenDrain>, stm32f1xx_hal::gpio::CRL, 'B', 7>
            )
        >,
        delay: Delay<TIM4, 10000>,
        led: Pin<Output<PushPull>, CRH, 'C', 13>,
        proximity: Pin<Input<PullUp>, CRL, 'B', 4>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp: stm32::Peripherals = cx.device;
        let rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let mut afio = dp.AFIO.constrain();

        // GPIO
        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();
        let mut gpioc = dp.GPIOC.split();

        let (_, _, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);


        // Configure the clock
        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz())
            .freeze(&mut flash.acr);

        // I2C configuration
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 5000.Hz(),
                duty_cycle: DutyCycle::Ratio16to9,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );
        let delay = dp.TIM4.delay::<10000>(&clocks);
        // End of I2C configuration

        // Proximity Sensor configuration
        let mut proximity = pb4.into_pull_up_input(&mut gpiob.crl);
        proximity.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        proximity.make_interrupt_source(&mut afio);
        proximity.enable_interrupt(&mut dp.EXTI);
        // End of Proximity Sensor configuration

        // LED configuration
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_low();
        led.toggle();


        // Push Button configuration
        let mut spool_change = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);
        spool_change.make_interrupt_source(&mut afio);
        spool_change.enable_interrupt(&mut dp.EXTI);
        let mut fast_forward = gpiob.pb0.into_pull_up_input(&mut gpiob.crl);
        fast_forward.make_interrupt_source(&mut afio);
        fast_forward.enable_interrupt(&mut dp.EXTI);
        let mut reverse = gpioa.pa2.into_pull_up_input(&mut gpioa.crl);
        reverse.make_interrupt_source(&mut afio);
        reverse.enable_interrupt(&mut dp.EXTI);
        // End of Push Button configuration


        (Shared {}, Local { i2c, delay, led, proximity }, init::Monotonics())
    }

    #[task(binds = EXTI4, local = [led, proximity])]
    fn exti4(_cx: exti4::Context) {
        let led = _cx.local.led;
        let proximity = _cx.local.proximity;
        led.toggle();
        PULSE_COUNT.fetch_add(1, Ordering::Relaxed);
        proximity.clear_interrupt_pending_bit();
    }



    #[idle(local = [i2c, delay])]
    fn idle(cx: idle::Context) -> ! {
        const LCD_ADDRESS: u8 = 0x27;

        let i2c = cx.local.i2c;
        let delay = cx.local.delay;
        let mut lcd = Lcd::new(i2c, delay)
            .address(LCD_ADDRESS)
            .cursor_on(true) // no visible cursors
            .rows(2) // two rows
            .init().unwrap();
        lcd.clear().unwrap();
        lcd.set_cursor(0, 0).unwrap();


        loop {
            let val = PULSE_COUNT.load(Ordering::Relaxed);
            // Write the value to the LCD
            let mut str_op: String<16> = String::new();
            write!(str_op, "Length::{:.2}", pulse_count_to_length(val as u32)).unwrap();
            lcd.clear().unwrap();
            lcd.write_str(str_op.as_str()).unwrap();
            lcd.set_cursor(0, 0).unwrap();


            // Toggle LED


            cortex_m::asm::wfi();
        }
    }
}

