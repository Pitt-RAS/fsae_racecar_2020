#ifndef SOFTWATCHDOG_H
#define SOFTWATCHDOG_H

class SoftWatchdog {
public:
    SoftWatchdog(unsigned int max_age_ms);
    void Feed();
    bool Hungry();
private:
    unsigned long last_fed_;
    unsigned int max_age_;
};

#endif
