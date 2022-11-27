#define ADC_THROTTLE_IN 0
#define HALL_SENSOR_IN  3
#define PWM_PULSE_OUT   5
#define GEAR_PULSE_OUT  7
#define GEAR_FIRST_IN   9
#define GEAR_SECOND_IN  10

#define ADC_VAL_MAX 192
#define ADC_VAL_MIN 65

#define RPM_VAL_MAX 630
#define RPM_VAL_MIN 0

uint32_t cnt_lim_u32 = 200;
uint32_t cnt_val_u32 = 0;

uint8_t adc_val_u8  = 0;
uint8_t hall_val_u8 = 0;
uint8_t pwm_val_u8  = 0;

uint32_t start_time_u32 = 0;
uint32_t prev_time_u32  = 0;
uint32_t pulse_time_u32 = 0;
uint32_t rpm_val_u32    = 0;

void setup()
{
  gpio_init();
  gear_config_func();
}

void loop()
{
  adc_read_func();
  softstart_func();
  pwm_write_func();
}

void gpio_init(void)
{
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_IN), hall_read_func, RISING);
  pinMode(HALL_SENSOR_IN, INPUT_PULLUP);
  pinMode(PWM_PULSE_OUT,  OUTPUT);
  pinMode(GEAR_PULSE_OUT, OUTPUT);
  pinMode(GEAR_FIRST_IN,  INPUT);
  pinMode(GEAR_SECOND_IN, INPUT);
  digitalWrite(GEAR_PULSE_OUT, LOW);
}

void adc_read_func(void)
{
  adc_val_u8 = map(analogRead(ADC_THROTTLE_IN), 0, 1023, 0, 255);
  boundary_func(adc_val_u8);
}

void hall_read_func(void)
{
  start_time_u32 = micros();
  pulse_time_u32 = start_time_u32 - prev_time_u32;
  rpm_val_u32    = (1000000/(pulse_time_u32/3))*60/72;
  prev_time_u32  = start_time_u32;
  hall_val_u8    = map(rpm_val_u32, RPM_VAL_MIN, RPM_VAL_MAX, ADC_VAL_MIN, ADC_VAL_MAX);
}

void pwm_write_func(void)
{
  boundary_func(pwm_val_u8);
  analogWrite(PWM_PULSE_OUT, pwm_val_u8);
}

void gear_config_func(void)
{
  if(digitalRead(GEAR_FIRST_IN))
  {
    pin_toggle_func(2);
  }
  else if(digitalRead(GEAR_SECOND_IN))
  {
    pin_toggle_func(1);
  }
}

void pin_toggle_func(uint8_t cnt)
{
  bool val = HIGH;
  for(uint8_t i = 0; i < 2 * cnt; i++)
  {
    digitalWrite(GEAR_PULSE_OUT, val);
    if(i < (2 * cnt - 1))
    {
      delay(300);
      val = !val;
    }
  }
}

void softstart_func(void)
{
  if(hall_val_u8 < adc_val_u8)
  {
    if(cnt_val_u32 >= cnt_lim_u32)
    {
      pwm_val_u8++;
      cnt_val_u32 = 0;
    }
    else
    {
      cnt_val_u32++;
    }
  }
  else
  {
    pwm_val_u8 = adc_val_u8;
  } 
}

void boundary_func(uint8_t val)
{
  if(ADC_VAL_MIN >= val)
  {
    val = ADC_VAL_MIN;
  }
  else if(ADC_VAL_MAX <= val)
  {
    val = ADC_VAL_MAX;
  }
}
