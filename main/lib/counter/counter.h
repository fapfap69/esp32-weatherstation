#ifndef COUNTER_H
#define COUNTER_H


#define MAXCOUNTERLIMIT 32767
#define MINCOUNTERLIMIT -32768

// --- FILTER  APB_CLK period 12.5ns (80MHz)
#define MICROSECTICKS 8

#include "driver/pcnt.h"

//#include <Arduino.h>


class Counter
{
  protected :
    typedef struct {
      int unit;  // the PCNT unit that originated an interrupt
      uint32_t status; // information on the event type that caused the interrupt
      void (* callBack)(int16_t) ;
      pcnt_evt_type_t eventType;
    } pcnt_evtDescriptor_t;

  protected:
    static pcnt_unit_t vMap[PCNT_UNIT_MAX];
    static bool vAssigned[PCNT_UNIT_MAX];
    static int assignedCounters;

    static pcnt_isr_handle_t user_isr_handle; //user's ISR service handle
    static void IRAM_ATTR pcnt_intr_handler(void *arg);
    static pcnt_evtDescriptor_t regInterrupts[PCNT_UNIT_MAX][4];

    pcnt_unit_t myUnit;
    pcnt_config_t myConfig0;
    pcnt_config_t myConfig1;
    pcnt_count_mode_t myMode0;
    pcnt_count_mode_t myMode1;


  public:
    Counter();
    Counter(int sig0_GPIO, int sig1_GPIO, int ctrl_GPIO, pcnt_count_mode_t mode);
    ~Counter();
    void start();
    void pause();
    void resume();
    int16_t getValue();

    void setFilter(bool enable, uint16_t uSeconds=0);
    uint16_t getFilter();

    void setEdge(bool positive=true);
    bool getEdge();

	  void setBoundaries(int16_t minimum, int16_t maximum);
	  void getBoundaries(int16_t *minimum, int16_t *maximum);

	  void setThreshHandler(int16_t threshold_0, int16_t threshold_1, void (* callBack)(int16_t));
	  void setZeroHandler(void (* callBack)(int16_t));
	  void setOverflowHandler(void (* callBack)(int16_t));

  protected:
    void init(int signal0Pin, int signal1Pin, int ctrlPin);
    void postInit();
    bool allocCounter();
    void freeCounter();

};

class CounterUp : public Counter
{
  public:
  	CounterUp(int sigGPIO);
  	~CounterUp();
};

class CounterDown : public Counter
{
  public:
  	CounterDown(int sigGPIO);
  	~CounterDown();
};


class CounterUpStrobe : public Counter
{
  public:
    CounterUpStrobe(int sigGPIO, int strobeGPIO);
  	~CounterUpStrobe();
  protected:
    void postInit();

};

class CounterDownStrobe : public Counter
{
  public:
  	CounterDownStrobe(int sigGPIO, int strobeGPIO);
  	~CounterDownStrobe();
  protected:
    void postInit();
};

class CounterUpDown : public Counter
{
  public:
    CounterUpDown(int sigGPIO, int updownGPIO);
	  ~CounterUpDown();
  protected:
    void postInit();
};

class CounterIncDec : public Counter
{
  public:
    CounterIncDec(int incGPIO, int decGPIO);
	  ~CounterIncDec();
  protected:
    void postInit();
};

class CounterIncDecStrobe : public Counter
{
  public:
    CounterIncDecStrobe(int incGPIO, int decGPIO, int strobeGPIO);
	  ~CounterIncDecStrobe();
  protected:
    void postInit();
};




#endif
