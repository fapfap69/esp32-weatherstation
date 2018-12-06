#include "counter.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
static const char* TAG = "Counter";

pcnt_unit_t Counter::vMap[PCNT_UNIT_MAX] = {
    PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3,
    PCNT_UNIT_4, PCNT_UNIT_5, PCNT_UNIT_6,PCNT_UNIT_7
    };

bool Counter::vAssigned[PCNT_UNIT_MAX] = {
    false,false,false,false,false,false,false,false };

int Counter::assignedCounters = 0;
pcnt_isr_handle_t Counter::user_isr_handle = NULL;

Counter::Counter() {}
Counter::Counter(int sig0_GPIO, int sig1_GPIO, int ctrl_GPIO, pcnt_count_mode_t mode)
{
  ESP_LOGD(TAG, "Create Counter (%d,%d,%d).", sig0_GPIO, sig1_GPIO, ctrl_GPIO);
  myUnit = ((pcnt_unit_t)-1);
  myMode0 = mode; // Increment/Decrement Counter
  myMode1 = PCNT_COUNT_DEC;

  if (allocCounter()) {
    init(sig0_GPIO, sig1_GPIO, ctrl_GPIO);
    postInit();
  }
}

Counter::~Counter()
{
  freeCounter();
}

void Counter::init(int sig0_GPIO, int sig1_GPIO, int ctrl_GPIO)
{
  pcnt_counter_pause(myUnit);

  /* Prepare configuration for the PCNT unit */
  // Set PCNT input signal and control GPIOs
  myConfig0.pulse_gpio_num = sig0_GPIO;
  myConfig0.ctrl_gpio_num = ctrl_GPIO;
  myConfig0.channel = PCNT_CHANNEL_0;
  myConfig0.unit = myUnit;
  // What to do on the positive / negative edge of pulse input?
  myConfig0.pos_mode = myMode0;   // Count up on the positive edge
  myConfig0.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
  // What to do when control input is low or high?
  myConfig0.lctrl_mode = PCNT_MODE_KEEP; // Reverse counting direction if low
  myConfig0.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
  // Set the maximum and minimum limit values to watch
  myConfig0.counter_h_lim = MAXCOUNTERLIMIT;
  myConfig0.counter_l_lim = MINCOUNTERLIMIT;

  myConfig1.pulse_gpio_num = sig1_GPIO;
  myConfig1.ctrl_gpio_num = ctrl_GPIO;
  myConfig1.channel = PCNT_CHANNEL_1;
  myConfig1.unit = myUnit;
  // What to do on the positive / negative edge of pulse input?
  myConfig1.pos_mode = myMode1;   // Count up on the positive edge
  myConfig1.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
  // What to do when control input is low or high?
  myConfig1.lctrl_mode = PCNT_MODE_KEEP; // Reverse counting direction if low
  myConfig1.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
  // Set the maximum and minimum limit values to watch
  myConfig1.counter_h_lim = MAXCOUNTERLIMIT;
  myConfig1.counter_l_lim = MINCOUNTERLIMIT;

}

void Counter::postInit()
{
  return;
}


void Counter::start()
{
  esp_err_t err;
  /* Initialize PCNT unit */
  err = pcnt_unit_config(&myConfig0);
  if (myConfig1.pulse_gpio_num != PCNT_PIN_NOT_USED)
    err = pcnt_unit_config(&myConfig1);

  err = pcnt_counter_clear(myUnit);
  err = pcnt_counter_resume(myUnit);
  return;
}

void Counter::pause()
{
  esp_err_t err;
  err = pcnt_counter_pause(myUnit);
  return;
}

void Counter::resume()
{
  esp_err_t err;
  err = pcnt_counter_resume(myUnit);
  return;
}


int16_t Counter::getValue()
{
  int16_t count;
  esp_err_t err;
  err = pcnt_get_counter_value(myUnit,&count);
  return count;
}

// --- FILTER
uint16_t Counter::getFilter()
{
  esp_err_t err;
  uint16_t value = 0;
  err = pcnt_get_filter_value(myUnit,&value);
  value = value / MICROSECTICKS;
  return value;
}
void Counter::setFilter(bool enable, uint16_t uSeconds)
{
  esp_err_t err;
  if (enable) {
    err = pcnt_filter_enable(myUnit);
    uint16_t filterVal =  uSeconds * MICROSECTICKS;
    err = pcnt_set_filter_value(myUnit,filterVal);
  } else {
    err = pcnt_filter_disable(myUnit);
  }
  return;
}

// --- EDGE
void Counter::setEdge(bool positive)
{
  if (positive) {
    myConfig0.pos_mode = myMode0;   // Count up on the positive edge
    myConfig0.neg_mode = PCNT_COUNT_DIS;
    myConfig1.pos_mode = myMode1;   // Count up on the positive edge
    myConfig1.neg_mode = PCNT_COUNT_DIS;
  } else {
    myConfig0.pos_mode = PCNT_COUNT_DIS;   // Count up on the negative edge
    myConfig0.neg_mode = myMode0;
    myConfig1.pos_mode = PCNT_COUNT_DIS;   // Count up on the negative edge
    myConfig1.neg_mode = myMode1;
  }
  return;
}

bool Counter::getEdge()
{
  if (myConfig0.pos_mode != PCNT_COUNT_DIS) {
    return true;
  } else if (myConfig0.neg_mode != PCNT_COUNT_DIS) {
    return false;
  } else {
    return true;
  }
}

// --- BOUNDARIES
void Counter::setBoundaries(int16_t minimum, int16_t maximum)
{
  myConfig0.counter_l_lim = minimum;
  myConfig0.counter_h_lim = maximum;
  myConfig1.counter_l_lim = minimum;
  myConfig1.counter_h_lim = maximum;
  return;
}

void Counter::getBoundaries(int16_t *minimum, int16_t *maximum)
{
  *minimum = myConfig0.counter_l_lim;
  *maximum = myConfig0.counter_h_lim;
  return;
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
void IRAM_ATTR Counter::pcnt_intr_handler(void *arg)
{
  uint32_t intr_status = PCNT.int_st.val;
  int i;
  pcnt_unit_t unit;
  pcnt_evt_type_t status;
  void (* cbFun)(int16_t) ;
  esp_err_t err;
  int16_t count;

  for (i = 0; i < PCNT_UNIT_MAX; i++) {
    if (intr_status & (BIT(i))) {
      unit = (pcnt_unit_t)i;
      status = (pcnt_evt_type_t)PCNT.status_unit[i].val;
      PCNT.int_clr.val = BIT(i);

  		err = pcnt_get_counter_value(unit,&count);
      switch(status) {
        case PCNT_EVT_THRES_1:
        case PCNT_EVT_THRES_0:
          cbFun = regInterrupts[unit][0].callBack;
          cbFun(count);
          break;
        case PCNT_EVT_ZERO:
          cbFun = regInterrupts[unit][1].callBack;
          cbFun(count);
        	break;
        case PCNT_EVT_H_LIM:
        case PCNT_EVT_L_LIM:
          cbFun = regInterrupts[unit][2].callBack;
          cbFun(count);
        	break;
        case PCNT_EVT_MAX:
          break;
      }
    }
  }
}

void Counter::setThreshHandler(int16_t threshold_0, int16_t threshold_1, void (* callBack)(int16_t) )
{
	regInterrupts[myUnit][0].unit = myUnit;
  regInterrupts[myUnit][0].status = 0;
  regInterrupts[myUnit][0].callBack = callBack;
  regInterrupts[myUnit][0].eventType = PCNT_EVT_THRES_0;

  /* Set threshold 0 and 1 values and enable events to watch */
  pcnt_set_event_value(myUnit, PCNT_EVT_THRES_1, threshold_1);
  pcnt_event_enable(myUnit, PCNT_EVT_THRES_1);
  pcnt_set_event_value(myUnit, PCNT_EVT_THRES_0, threshold_0);
  pcnt_event_enable(myUnit, PCNT_EVT_THRES_0);
  /* Register ISR handler and enable interrupts for PCNT unit */
  if(user_isr_handle == NULL) {
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(myUnit);
  }
}

void Counter::setZeroHandler(void (* callBack)(int16_t) )
{
	regInterrupts[myUnit][1].unit = myUnit;
  regInterrupts[myUnit][1].status = 0;
  regInterrupts[myUnit][1].callBack = callBack;
  regInterrupts[myUnit][1].eventType = PCNT_EVT_ZERO;

  /* Enable events on zero, maximum and minimum limit values */
  pcnt_event_enable(myUnit, PCNT_EVT_ZERO);

  /* Register ISR handler and enable interrupts for PCNT unit */
  if(user_isr_handle == NULL) {
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(myUnit);
  }
}
void Counter::setOverflowHandler(void (* callBack)(int16_t) )
{
	regInterrupts[myUnit][2].unit = myUnit;
  regInterrupts[myUnit][2].status = 0;
  regInterrupts[myUnit][2].callBack = callBack;
  regInterrupts[myUnit][2].eventType = PCNT_EVT_L_LIM;

  /* Enable events on zero, maximum and minimum limit values */
  pcnt_event_enable(myUnit, PCNT_EVT_H_LIM);
  pcnt_event_enable(myUnit, PCNT_EVT_L_LIM);

  /* Register ISR handler and enable interrupts for PCNT unit */
  if(user_isr_handle == NULL) {
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(myUnit);
  }
}

bool Counter::allocCounter()
{
  for (int i=0;i<PCNT_UNIT_MAX;i++) {
    if (vAssigned[i] == false) {
      vAssigned[i] = true;
      myUnit = vMap[i];
      assignedCounters++;
      return true;
    }
  }
  return false;
}

void Counter::freeCounter()
{
  for (int i=0;i<PCNT_UNIT_MAX;i++) {
    if (vMap[i] == myUnit) {
      vAssigned[i] = false;
      myUnit = ((pcnt_unit_t)-1);
      assignedCounters--;

      pcnt_intr_disable(myUnit);
      pcnt_event_disable(myUnit, PCNT_EVT_THRES_0);
      pcnt_event_disable(myUnit, PCNT_EVT_THRES_1);
      pcnt_event_disable(myUnit, PCNT_EVT_ZERO);
      pcnt_event_disable(myUnit, PCNT_EVT_H_LIM);
      pcnt_event_disable(myUnit, PCNT_EVT_L_LIM);

      break;
    }
  }
  return;
}

// -------------------------------
CounterUp::CounterUp(int sigGPIO):Counter(sigGPIO,PCNT_PIN_NOT_USED,PCNT_PIN_NOT_USED,PCNT_COUNT_INC) {}
CounterUp::~CounterUp() {freeCounter();}
// -------------------------------
CounterDown::CounterDown(int sigGPIO):Counter(sigGPIO,PCNT_PIN_NOT_USED,PCNT_PIN_NOT_USED,PCNT_COUNT_DEC) {}
CounterDown::~CounterDown() {freeCounter();}
// -----------------------
CounterUpStrobe::CounterUpStrobe(int sigGPIO, int strobeGPIO):Counter(sigGPIO,PCNT_PIN_NOT_USED,strobeGPIO,PCNT_COUNT_INC) {}
CounterUpStrobe::~CounterUpStrobe() {freeCounter();}

void CounterUpStrobe::postInit()
{
  myConfig0.lctrl_mode = PCNT_MODE_DISABLE; // Ignore signal if low
}
// -----------------------
CounterDownStrobe::CounterDownStrobe(int sigGPIO, int strobeGPIO):Counter(sigGPIO,PCNT_PIN_NOT_USED,strobeGPIO,PCNT_COUNT_DEC) {}
CounterDownStrobe::~CounterDownStrobe() {freeCounter();}

void CounterDownStrobe::postInit()
{
  myConfig0.lctrl_mode = PCNT_MODE_DISABLE; // Ignore signal if low
}
// -----------------------
CounterUpDown::CounterUpDown(int sigGPIO, int updownGPIO):Counter(sigGPIO,PCNT_PIN_NOT_USED,updownGPIO,PCNT_COUNT_INC) {}
CounterUpDown::~CounterUpDown() {freeCounter();}

void CounterUpDown::postInit()
{
  myConfig0.hctrl_mode = PCNT_MODE_REVERSE;    // Reverse the primary counter mode if low
}

// -----------------------
CounterIncDec::CounterIncDec(int incGPIO, int decGPIO):Counter(incGPIO,decGPIO,PCNT_PIN_NOT_USED,PCNT_COUNT_DIS) {}
CounterIncDec::~CounterIncDec() {freeCounter();}

void CounterIncDec::postInit()
{
  myConfig0.pos_mode = PCNT_COUNT_INC;  // Count up on the positive edge
  myConfig1.pos_mode = PCNT_COUNT_DEC;  // Count down on the positive edge
}
// -----------------------
CounterIncDecStrobe::CounterIncDecStrobe(int incGPIO, int decGPIO, int strobeGPIO):Counter(incGPIO,decGPIO,strobeGPIO,PCNT_COUNT_DIS) {}
CounterIncDecStrobe::~CounterIncDecStrobe() {freeCounter();}

void CounterIncDecStrobe::postInit()
{
  myConfig0.lctrl_mode = PCNT_MODE_DISABLE; // Ignore signal if low
  myConfig0.pos_mode = PCNT_COUNT_INC;      // Count up on the positive edge
  myConfig1.lctrl_mode = PCNT_MODE_DISABLE; // Ignore signal if low
  myConfig1.pos_mode = PCNT_COUNT_DEC;      // Count down on the positive edge
}
