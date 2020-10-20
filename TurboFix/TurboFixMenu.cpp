#include "ScriptMenu.hpp"
#include "Script.hpp"
#include "TurboScript.hpp"
#include "Constants.hpp"

#include "Memory/Patches.h"
#include "ScriptMenuUtils.h"

#include "Util/UI.hpp"

#include <fmt/format.h>

std::vector<CScriptMenu<CTurboScript>::CSubmenu> TurboFix::BuildMenu() {
    std::vector<CScriptMenu<CTurboScript>::CSubmenu> submenus;
    /* mainmenu */
    submenus.emplace_back("mainmenu", [](NativeMenu::Menu& mbCtx, CTurboScript& context) {
        mbCtx.Title("Turbo Fix");
        mbCtx.Subtitle(std::string("~b~") + Constants::DisplayVersion);

        if (mbCtx.BoolOption("Enable", context.Settings().Main.Enable,
            { "Enable or disable the entire script." })) {
            Patches::BoostLimiter(context.Settings().Main.Enable);
        }

        mbCtx.MenuOption("Configs", "configsmenu",
            { "An overview of configurations available." });

        CConfig* activeConfig = context.ActiveConfig();
        mbCtx.MenuOption(fmt::format("Active config: {}", activeConfig ? activeConfig->Name : "None"),
            "editconfigmenu",
            { "Enter to edit the current configuration." });

        mbCtx.MenuOption("Developer options", "developermenu");
        });

    /* mainmenu -> configsmenu */
    submenus.emplace_back("configsmenu", [](NativeMenu::Menu& mbCtx, CTurboScript& context) {
        mbCtx.Title("Configs");
        mbCtx.Subtitle("Overview");

        if (mbCtx.Option("Reload configs")) {
            context.LoadConfigs();
            context.UpdateActiveConfig(true);
        }

        for (const auto& config : context.Configs()) {
            bool selected;
            mbCtx.OptionPlus(config.Name, {}, &selected);

            if (selected) {
                std::vector<std::string> extras{
                    fmt::format("Models: {}", fmt::join(config.ModelNames, ", ")),
                    fmt::format("Plates: {}", fmt::join(config.Plates, ", ")),

                    fmt::format("RPM Spool Start: {:.2f}", config.RPMSpoolStart),
                    fmt::format("RPM Spool End: {:.2f}", config.RPMSpoolEnd),
                    fmt::format("Min boost: {:.2f}", config.MinBoost),
                    fmt::format("Max boost: {:.2f}", config.MaxBoost),
                    fmt::format("Spool rate: {:.5f}", config.SpoolRate),
                    fmt::format("Unspool rate: {:.5f}", config.UnspoolRate),
                    fmt::format("Anti-lag: {}", config.AntiLag),
                };

                mbCtx.OptionPlusPlus(extras);
            }
        }
        });

    /* mainmenu -> editconfigmenu */
    submenus.emplace_back("editconfigmenu", [](NativeMenu::Menu& mbCtx, CTurboScript& context) {
        mbCtx.Title("Config edit");
        CConfig* config = context.ActiveConfig();
        mbCtx.Subtitle(config ? config->Name : "None");

        if (config == nullptr) {
            mbCtx.Option("No active configuration");
            return;
        }

        mbCtx.FloatOptionCb("RPM Spool Start", config->RPMSpoolStart, 0.0f, 1.0f, 0.01f, MenuUtils::GetKbFloat,
            { "At what RPM the turbo starts building boost.",
              "0.2 RPM is idle." });

        mbCtx.FloatOptionCb("RPM Spool End", config->RPMSpoolEnd, 0.0f, 1.0f, 0.01f, MenuUtils::GetKbFloat,
            { "At what RPM the turbo boost is maximal.",
              "1.0 RPM is rev limit." });

        mbCtx.FloatOptionCb("Min boost", config->MinBoost, -1000000.0f, 0.0f, 0.01f, MenuUtils::GetKbFloat,
            { "What the max vacuum is, e.g. when closing the throttle at high RPM.",
              "Keep this at a similar amplitude to max boost."});

        mbCtx.FloatOptionCb("Max boost", config->MaxBoost, 0.0f, 1000000.0f, 0.01f, MenuUtils::GetKbFloat,
            { "What full boost is. A value of 1.0 adds 10% of the current engine power." });

        mbCtx.FloatOptionCb("Spool rate", config->SpoolRate, 0.01f, 0.999999f, 0.00005f, MenuUtils::GetKbFloat,
            { "How fast the turbo spools up, in part per 1 second.",
              "So 0.5 is it spools up to half its max after 1 second.",
              "0.999 is almost instant. Keep under 1.0." });

        mbCtx.FloatOptionCb("Unspool rate", config->UnspoolRate, 0.01f, 0.999999f, 0.00005f, MenuUtils::GetKbFloat,
            { "How fast the turbo slows down. Calculation is same as above." });

        mbCtx.MenuOption("Anti-lag settings", "antilagsettingsmenu",
            { "Anti-lag keeps the turbo spinning when off-throttle at higher RPMs." });

        mbCtx.MenuOption("Dial settings", "dialsettingsmenu", 
            { "Remap the turbo dial on vehicle dashboards.",
              "DashHook and a vehicle with working boost gauge are required for this feature." });

        if (mbCtx.Option("Save changes")) {
            config->Write();
            UI::Notify("Saved changes", true);
            context.LoadConfigs();
            context.UpdateActiveConfig(true);
        }

        if (mbCtx.Option("Save as...")) {
            UI::Notify("Enter new config name.", true);
            std::string newName = UI::GetKeyboardResult();

            UI::Notify("Enter model(s).", true);
            std::string newModel = UI::GetKeyboardResult();

            if (newName.empty() || newModel.empty()) {
                UI::Notify("No config name or model name entered. Not saving anything.", true);
                return;
            }

            if (config->Write(newName, newModel))
                UI::Notify("Saved as new configuration", true);
            else
                UI::Notify("Failed to save as new configuration", true);
            context.LoadConfigs();
            context.UpdateActiveConfig(true);
        }
        });

    /* mainmenu -> editconfigmenu -> dialsettingsmenu */
    submenus.emplace_back("dialsettingsmenu", [](NativeMenu::Menu& mbCtx, CTurboScript& context) {
        mbCtx.Title("Dial adjustment");
        CConfig* config = context.ActiveConfig();
        mbCtx.Subtitle(config ? config->Name : "None");

        if (config == nullptr) {
            mbCtx.Option("No active configuration");
            return;
        }

        mbCtx.FloatOptionCb("Dial offset (boost)", config->DialBoostOffset, -10.0f, 10.0f, 0.05f, MenuUtils::GetKbFloat,
            { "Starting offset of the boost dial. Press Enter to manually enter a number." });

        mbCtx.FloatOptionCb("Dial scale (boost)", config->DialBoostScale, -10.0f, 10.0f, 0.05f, MenuUtils::GetKbFloat,
            { "Scaling of the boost dial. Press Enter to manually enter a number." });

        mbCtx.FloatOptionCb("Dial offset (vacuum)", config->DialVacuumOffset, -10.0f, 10.0f, 0.05f, MenuUtils::GetKbFloat,
            { "Starting offset of the vacuum dial. Press Enter to manually enter a number." });

        mbCtx.FloatOptionCb("Dial scale (vacuum)", config->DialVacuumScale, -10.0f, 10.0f, 0.05f, MenuUtils::GetKbFloat,
            { "Scaling of the vacuum dial. Press Enter to manually enter a number." });

        mbCtx.BoolOption("Dial boost includes vacuum", config->DialBoostIncludesVacuum,
            { "Remap vacuum data to the boost dial, for combined vacuum and boost dials. Vacuum offset is ignored." });
        });

    /* mainmenu -> editconfigmenu -> antilagsettingsmenu */
    submenus.emplace_back("antilagsettingsmenu", [](NativeMenu::Menu& mbCtx, CTurboScript& context) {
        mbCtx.Title("Anti-lag");
        CConfig* config = context.ActiveConfig();
        mbCtx.Subtitle(config ? config->Name : "None");

        if (config == nullptr) {
            mbCtx.Option("No active configuration");
            return;
        }

        mbCtx.BoolOption("Enable", config->AntiLag);
        mbCtx.BoolOption("Effects", config->AntiLagEffects, { "Exhaust pops, bangs and fire." });
        if (mbCtx.StringArray("Sound set", context.GetSoundSets(), context.SoundSetIndex())) {
            config->AntiLagSoundSet = context.GetSoundSets()[context.SoundSetIndex()];
        }
        mbCtx.IntOption("Loud duration (ticks)", config->AntiLagSoundTicks, 0, 100, 1);
        mbCtx.FloatOptionCb("Volume", config->AntiLagSoundVolume, 0.0f, 2.0f, 0.05f, MenuUtils::GetKbFloat);
        });

    /* mainmenu -> developermenu */
    submenus.emplace_back("developermenu", [](NativeMenu::Menu& mbCtx, CTurboScript& context) {
        mbCtx.Title("Developer options");
        mbCtx.Subtitle("");

        mbCtx.Option(fmt::format("NPC instances: {}", TurboFix::GetNPCScriptCount()));
        mbCtx.BoolOption("NPC Details", context.Settings().Debug.NPCDetails);
        });

    return submenus;
}
