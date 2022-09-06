#ifndef PRIENV_H
#define PRIENV_H
#include  "Env.hpp"

class PriEnv:public CoreEnv
{
public:
    virtual unsigned long episodeReset(bool manual_reset);
    PriEnv(std::string task_name, uint16_t ob_size, uint16_t act_size, uint16_t step_limit);
    ~PriEnv();
    virtual void getOb();
};

#endif // PRIENV_H
