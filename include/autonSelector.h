#pragma once
#include "vex.h"

class AutonSelector {
private:
    int currentAutonIndex;
    bool selecting;
    bool confirmed;
    
    vex::task* selectionTask;
    static int selectionTaskFunc(void* selector);
    void updateSelection();
    
public:
        struct AutonOption {
        const char* name;
        void (*autonFunction)();
    };
    
    AutonSelector();
    void addAuton(const char* name, void (*autonFunction)());
    void startSelection();
    void stopSelection();
    void executeSelectedAuton();
    bool isSelecting() { return selecting; }
};

