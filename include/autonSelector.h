#pragma once
#include "vex.h"

class AutonSelector {
private:
    int currentAutonIndex;
    bool selecting;
    bool confirmed;
    
    struct AutonOption {
        const char* name;
        void (*autonFunction)();
    };
    
    std::vector<AutonOption> autonOptions;
    
public:
    AutonSelector();
    void addAuton(const char* name, void (*autonFunction)());
    void displayAutonSelection();
    void runSelection();
    void executeSelectedAuton();
};