#include "EncoderPublisher.h"
#include <Arduino.h>
#include "config.h"

static bool encoder_isr_init = false;

// these are all totals
volatile static long int encoder_count_A = 0;
volatile static long int encoder_count_B = 0;
volatile static long int encoder_count_C = 0;

volatile static int last_isr = 0;                           // 1=A, 2=B, 3=C - Using this to check last phase

static long int encoder_total = 0;                          // A+B+C
static long int encoder_last_total = 0;
static int encoder_delta = 0;

// forwards would go A C B A C B ...
// backwards would go A B C A B C ...
static void isr_A(){
    // if last phase triggered was B, increment; else decrement (because we went backwards)
    if (last_isr == 2){
        encoder_count_A++;
    }
    else{
        encoder_count_A--;
    }
    
    last_isr = 1;
}

static void isr_B(){
    // if last phase triggered was C, increment; else decrement (backwards)
    if (last_isr == 3){
        encoder_count_B++;
    }
    else{
        encoder_count_B--;
    }

    last_isr = 2;
}

static void isr_C(){
    // if last phase triggered was A, increment; else decrement (backwards)
    if (last_isr == 1){
        encoder_count_C++;
    }
    else{
        encoder_count_C--;
    }

    last_isr = 3;
}

static void SetupEncoderISR() {
    if ( encoder_isr_init )
        return;
    encoder_isr_init = true;

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(ENCODER_C, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), isr_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), isr_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_C), isr_C, CHANGE);
}

EncoderPublisher::EncoderPublisher(ros::NodeHandle& nh) :
        nh_(nh), 
        velocity_publisher_("/platform/velocity", &twist_msg_),                     
        encoder_count_publisher_("/platform/encoder_count", &encoder_count_msg_),   
        update_rate_(ENCODER_UPDATE_HZ),                    
        update_rate_debug_(DEBUG_HZ),                       
        last_count_A(0), 
        last_count_B(0),
        last_count_C(0) {
    SetupEncoderISR();

    nh.advertise(velocity_publisher_);                      
    nh.advertise(encoder_count_publisher_);                 

    twist_msg_.header.frame_id = "base_link";               
    twist_msg_.twist.covariance[0] = kVelocityVariance;     
}

void EncoderPublisher::Update(bool noop) {
    if ( update_rate_.NeedsRun() ) {
        // Update encoder counts
        cli();                                              // stop isrs

        encoder_total = encoder_count_A + encoder_count_B + encoder_count_C;
        encoder_delta = encoder_total - encoder_last_total;
        encoder_last_total = encoder_total;
        
        sei();                                              // start isrs

        // TODO: Publish correct information to correct topics
        double velocity = compute_velocity(encoder_delta);
        twist_msg_.header.stamp = nh_.now();
        twist_msg_.twist.twist.linear.x = velocity;

        velocity_publisher_.publish(&twist_msg_);
    }

    if ( update_rate_debug_.NeedsRun() ) {
        encoder_count_msg_.left = encoder_total;
        encoder_count_msg_.right = -1;
        encoder_count_publisher_.publish(&encoder_count_msg_);
    }
}

// pass encoder_total
double EncoderPublisher::compute_distance(int steps){
    double operand = steps;
    operand = operand / STEPS_PER_REV;                  // 6 steps per revolution (motor)
    operand = operand / GEAR_RATIO;            // gear ratio (87 teeth on big gear / 18 teeth on small gear)
    operand = operand / BIG_GEAR_TO_WHEEL_RATIO;                  // 3 big gear revolutions per 1 wheel revolution
    // at this point operand = wheel revolutions in timeframe

    operand = operand * (PI * WHEEL_DIAMETER_METERS);   // total distance driven
    return operand;
}

// pass encoder_delta
float EncoderPublisher::compute_velocity(int delta){
    float operand = delta;
    operand = operand / STEPS_PER_REV;         // 6 operand per revolution (motor)
    operand = operand / GEAR_RATIO; // gear ratio (87 teeth on big gear / 18 teeth on small gear)
    operand = operand / BIG_GEAR_TO_WHEEL_RATIO;         // 3 big gear revolutions per 1 wheel revolution
    // at this point operand = wheel revolutions in timeframe

    operand = operand * (PI * WHEEL_DIAMETER_METERS); // distance driven from (t2 - t1)
    operand = operand * ENCODER_UPDATE_HZ;        // operand = inches/second
    return operand;
}
