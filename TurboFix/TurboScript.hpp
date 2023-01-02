#pragma once
#include "ScriptSettings.hpp"
#include "Config.hpp"
#include "SoundSet.hpp"

#include "Memory/VehicleExtensions.hpp"

#include <irrKlang.h>
#include <vector>
#include <string>

class CTurboScript {
public:
    CTurboScript(
        CScriptSettings& settings,
        std::vector<CConfig>& configs,
        std::vector<SSoundSet>& soundSets);
    virtual ~CTurboScript();
    virtual void Tick();

    CConfig* ActiveConfig() {
        return mActiveConfig;
    }

    bool GetHasTurbo();
    float GetCurrentBoost();

    void UpdateActiveConfig(bool playerCheck);

    // Applies the passed config onto the current active config.
    void ApplyConfig(const CConfig& config);

    int& SoundSetIndex() {
        return mSoundSetIndex;
    }

    Vehicle GetVehicle() {
        return mVehicle;
    }

protected:
    void runPtfx(Vehicle vehicle, bool loud);
    void runSfx(Vehicle vehicle, bool loud);
    float updateAntiLag(float currentBoost, float newBoost, float limBoost);
    void updateDial(float newBoost);
    void updateTurbo();
    void updateSoundSetIndex(const std::string& soundSet);

    const CScriptSettings& mSettings;
    std::vector<CConfig>& mConfigs;
    CConfig mDefaultConfig;

    Vehicle mVehicle;
    CConfig* mActiveConfig;

    int mLastFxTime;
    int mLastLoudTime;

    float mLastThrottle;

    const std::vector<SSoundSet>& mSoundSets;
    int mSoundSetIndex;

    irrklang::ISoundEngine* mSoundEngine;
    const std::vector<std::string> mExhaustBones = {
        "exhaust",    "exhaust_2",  "exhaust_3",  "exhaust_4",
        "exhaust_5",  "exhaust_6",  "exhaust_7",  "exhaust_8",
        "exhaust_9",  "exhaust_10", "exhaust_11", "exhaust_12",
        "exhaust_13", "exhaust_14", "exhaust_15", "exhaust_16",
        "exhaust_17", "exhaust_18", "exhaust_19", "exhaust_20",
        "exhaust_21", "exhaust_22", "exhaust_23", "exhaust_24",
        "exhaust_25", "exhaust_26", "exhaust_27", "exhaust_28",
        "exhaust_29", "exhaust_30", "exhaust_31", "exhaust_32",
    };

    bool mIsNPC;
};
