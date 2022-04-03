/*
 Author: Rajesh Pachaikani
 Github: github.com/rajeshpachaikani

 Project: Rust-stepper-arm
 Description: This is a rust program to control two stepper motors using STM32 Blue Pill.
 One of the stepper motor runs continuously in the forward direction and the other motor runs
 until a limit switch is reached and then reverses the direction.

 Pin Configuration:
    PA1 - Step pin for motor 1 uses TIM2
    PA6 - Step pin for motor 2 uses TIM3
    PB12 - Direction pin for motor 1
    PB13 - Direction pin for motor 2
    PB1 - Limit switch pin

*/


#![no_std]
#![no_main]

use cortex_m::asm::delay;
use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::syscall;
use stm32f1xx_hal::gpio::ExtiPin;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::timer::{pwm,Channel, PwmHz, Tim2NoRemap, Tim3NoRemap};

//Get Pulse Count
fn get_pulse_count(rpm: u32, ustep: u8) -> u32 {
    ((rpm as f32/60 as f32)*200 as f32* ustep as f32) as u32
}

#[entry]
fn main() -> ! {
    // Initialize the peripherals
    let dp = stm32f1xx_hal::pac::Peripherals::take().unwrap();
    // let cp = cortex_m::Peripherals::take().unwrap();
    let mut afio = dp.AFIO.constrain();

    //Set clock source to HSI
    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz())
        .sysclk(48.MHz())
        .freeze(&mut flash.acr);

    //Stepper Enable pin (PB11)
    let mut gpio_b = dp.GPIOB.split();
    let mut gpio_c = dp.GPIOC.split();
    let mut led = gpio_c.pc13.into_push_pull_output(&mut gpio_c.crh);
    let mut en = gpio_b.pb11.into_push_pull_output(&mut gpio_b.crh);
    // en.set_low();
    en.set_low();
    let mut gpio_a = dp.GPIOA.split();
    let mut dir_1 = gpio_b.pb12.into_push_pull_output(&mut gpio_b.crh);
    dir_1.set_high();
    let mut dir_2 = gpio_b.pb12.into_push_pull_output(&mut gpio_b.crh);
    dir_2.set_high();



    //Initialize timer 2 for PWM
    let c1 = gpio_a.pa1.into_alternate_push_pull(&mut gpio_a.crl);
    let c2 = gpio_a.pa2.into_alternate_push_pull(&mut gpio_a.crl);
    let pins = (c1,c2);
    let mut pwm_1 = dp.TIM2.pwm_hz::<Tim2NoRemap, _,_>(pins,&mut afio.mapr,10.kHz(),&clocks);


    //Initialize timer 3 for PWM
    let c3 = gpio_a.pa6.into_alternate_push_pull(&mut gpio_a.crl);
    let c4 = gpio_a.pa7.into_alternate_push_pull(&mut gpio_a.crl);
    let pins = (c3,c4);
    let mut pwm_2 = dp.TIM3.pwm_hz::<Tim3NoRemap, _,_>(pins,&mut afio.mapr,10.kHz(),&clocks);

    //Todo: Use ADC to set RPM
    let rpm_set_point_1 = 360; // 380 RPM (MAX)
    let rpm_set_point_2 = 360; // 380 RPM (MAX)

    let mut current_speed = 100;
    pwm_1.enable(Channel::C2);
    pwm_1.enable(Channel::C3);
    pwm_1.set_period(get_pulse_count(rpm_set_point_1,1).Hz());
    let mx_duty_1 = pwm_1.get_max_duty();
    let mx_duty_2 = pwm_2.get_max_duty();
    pwm_1.set_duty(Channel::C2, mx_duty_1 /3);
    // let mut speed_var = 10; //Start from 10 rpm

    pwm_2.enable(Channel::C1);
    pwm_2.set_period(get_pulse_count(rpm_set_point_2,1).Hz());
    pwm_2.set_duty(Channel::C1, mx_duty_2 /3);
    //Initialize limit switch in PB1
    let mut limit_switch = gpio_b.pb0.into_pull_up_input(&mut gpio_b.crl);


    loop{
        if limit_switch.is_low() {
            dir_1.toggle();
            led.toggle();
            delay(10_000_000);
        }
    }
}
