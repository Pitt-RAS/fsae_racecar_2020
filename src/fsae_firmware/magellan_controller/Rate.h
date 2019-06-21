#ifndef RATE_H
#define RATE_H

class Rate {
public:
    Rate(int);
    bool NeedsRun();
    void Sleep();
    void SetRate(int rate);
private:
    unsigned long period_us_;
    unsigned long last_us_;
};

#endif
