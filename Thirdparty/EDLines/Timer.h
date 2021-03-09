#ifndef _TIMER_H_
#define _TIMER_H_

#include <sys/time.h>

class Timer {
private:
  struct timeval start, end;

public:
  Timer(){} //end-TimerClass

  void Start(){
    gettimeofday(&start, NULL);
  } //end-Start

  void Stop(){
    gettimeofday(&end, NULL);
  } //end-Stop

  // Returns time in milliseconds
  double ElapsedTime(){
    if (end.tv_sec == start.tv_sec) return (end.tv_usec - start.tv_usec)/1e3;
    double elapsedTime = 1e6-start.tv_usec;
    elapsedTime += end.tv_usec;
    elapsedTime += 1e6*(end.tv_sec-start.tv_sec-1);

    return elapsedTime/1e3;
  } //end-Elapsed
};

#endif