#!/system/bin/sh
# Open the Brake Indicator app from NEMESIS Manager (Launch on Brake Light Indicator),
# installing its APK first if needed. Needs no launcher hooks, so it opens the app on
# cars without an HMG-API hook map too.
MOD=${ROOT:-}/data/hmgapi/mods/brake-indicator-gen5w
say() { echo "Brake Indicator: $1"; }
[ "$(getprop sys.boot_completed)" = 1 ] || { say "Android isn't up yet: try again in a minute"; exit 1; }
case "$(pm path dev.lemoyne.brakeindicator 2>/dev/null)" in
    package:*) ;;
    *)
        A=$MOD/app/BrakeIndicator.apk
        [ -f "$A" ] || { say "the app isn't on this unit: install Brake Light Indicator again"; exit 1; }
        setprop sys.boot_completed 0
        r=$(pm install -r "$A" 2>&1)
        setprop sys.boot_completed 1
        case "$r" in
            *Success*) say "installed the app" ;;
            *) say "the app didn't install: $r"; exit 1 ;;
        esac ;;
esac
am start -n dev.lemoyne.brakeindicator/.MainActivity >/dev/null 2>&1 || { say "couldn't open the app"; exit 1; }
