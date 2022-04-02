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
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::timer::{pwm,Channel, PwmHz, Tim2NoRemap};

//Get Pulse Count
fn get_pulse_count(rpm: u32) -> u32 {
    ((rpm as f32/60 as f32)*200 as f32*1 as f32) as u32
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
    let mut dir = gpio_b.pb12.into_push_pull_output(&mut gpio_b.crh);
    dir.set_high();

    //Initialize timer 2 for PWM
    let c1 = gpio_a.pa1.into_alternate_push_pull(&mut gpio_a.crl);
    let c2 = gpio_a.pa2.into_alternate_push_pull(&mut gpio_a.crl);
    let pins = (c1, c2);
    let mut pwm_1 = dp.TIM2.pwm_hz::<Tim2NoRemap, _,_>(pins,&mut afio.mapr,10.kHz(),&clocks);
    let rpm_set_point = 380; // 380 RPM (MAX)
    let mut current_speed = 100;
    pwm_1.enable(Channel::C2);
    pwm_1.enable(Channel::C3);
    // loop{
    //     if current_speed >= rpm_set_point{
    //         break;
    //     }
    //     else if rpm_set_point > current_speed{
    //         current_speed += 20;
    //         pwm_1.set_period(get_pulse_count(current_speed).Hz());
    //         led.toggle();
    //     }
    //     delay(1_000_000);
    // }


    pwm_1.set_period(get_pulse_count(rpm_set_point).Hz());
    // pwm_1.enable(Channel::C2);
    // pwm_1.enable(Channel::C3);
    let mx_duty = pwm_1.get_max_duty();
    pwm_1.set_duty(Channel::C2,mx_duty/3);
    // let mut speed_var = 10; //Start from 10 rpm
    // dir.toggle();
    loop{

    }
}
