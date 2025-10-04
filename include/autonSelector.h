#pragma once
#include "vex.h"

enum class CompetitionAutonMode {
    SKILLS = 0,
    LEFT_AWP = 1,
    RIGHT_AWP = 2,
    LEFT_ELIM = 3, 
    RIGHT_ELIM = 4,
    SAFE = 5,
    DISABLED = 6
};

class CompetitionAutonSelector {
private:
    CompetitionAutonMode selectedMode;
    int currentIndex;
    bool competitionMode;
    
    const char* modeNames[7] = {
        "SKILLS", "LEFT AWP", "RIGHT AWP", "LEFT ELIM",
        "RIGHT ELIM", "SAFE", "DISABLED"
    };

    color modeColors[7] = {
        red, blue, blue, green, green, orange, black
    };

public:
    CompetitionAutonSelector();
    
    void initialize();
    void updateDisplays();
    void cycleForward();
    void cycleBackward();
    void update();
    
    void setCompetitionMode(bool isCompetition);
    CompetitionAutonMode getSelectedMode();
    const char* getModeName();
    
    void runSelectedAuton();
    
private:
    void updatePreMatchDisplay();
    void updateCompetitionDisplay();
    void updateControllerDisplay();
};
