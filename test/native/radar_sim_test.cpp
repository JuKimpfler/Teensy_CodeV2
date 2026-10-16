// Rechnertest fuer lib/RadarArray mit simulierten Sensoren: Messrunden, Rundenzaehler, Abfragelast, Einspeisung vom Link.
#include <RadarArray.h>

#include <algorithm>
#include <cstdio>

uint32_t g_us = 0;
long g_busReads = 0;
uint32_t g_dur[4] = {45000, 50000, 52000, 48000};
uint32_t g_finishUs[4];
int g_nextId = 0;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond)                                                    \
    do                                                                 \
    {                                                                  \
        g_checks++;                                                    \
        if (!(cond))                                                   \
        {                                                              \
            g_fail++;                                                  \
            printf("  FEHLER %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                              \
    } while (0)

int main()
{
    printf("RadarArray: Messrunden, Rundenzaehler, Abfragelast\n");
    for (uint32_t cycleUs : {1000u, 3000u, 10000u})
    {
        g_us = 0; // Zeit und Sensor-Nummern je Durchlauf zuruecksetzen, begin() laeuft bei t = 0
        g_nextId = 0;
        RadarArray radar;
        g_busReads = 0;
        radar.begin(Wire);
        long cycles = 0;
        uint32_t lastId = radar.giveRoundId();
        long rounds = 0, badMid = 0;
        uint32_t roundStart = g_us;
        for (; g_us < 10000000; g_us += cycleUs)
        {
            cycles++;
            radar.update();
            if (radar.giveRoundId() != lastId)
            {
                rounds += radar.giveRoundId() - lastId;
                lastId = radar.giveRoundId();
                // Mitte der Runde liegt zwischen Start und Ende (die Runde dauert ~52 ms)
                const uint32_t mid = radar.giveRoundMidUs();
                const uint32_t age = g_us - mid;
                if (!(age > 20000 && age < 45000))
                    badMid++;
                roundStart = g_us;
            }
        }
        (void)roundStart;
        const double polls = (double)g_busReads / cycles;
        printf("  Zyklus %5u us: %ld Runden in 10 s (%.1f/s), Mitte-Fehler %ld, Statusabfragen/Zyklus %.2f\n", cycleUs, rounds, rounds / 10.0, badMid, polls);
        CHECK(rounds >= 150 && rounds <= 200); // ~52 ms je Runde
        CHECK(badMid == 0);
        if (cycleUs <= 3000)
            CHECK(polls < 1.5); // ohne Verzoegerung der Abfrage waeren es ~3,7 je Zyklus
    }

    printf("RadarArray: Einspeisung vom Link\n");
    {
        RadarArray radar;
        g_us = 5000000;
        radar.feedRemote(RadarVo, 1000, 1010, -11000, true, true, 4990);
        radar.feedRemote(RadarRe, 700, 705, -12000, false, true, 4990);
        CHECK(radar[RadarVo].Dist == 1000 && radar[RadarVo].DistRaw == 1010 && radar[RadarVo].Strenght == -11000);
        CHECK(radar[RadarVo].Peaks == 1 && radar[RadarRe].Peaks == 0);
        CHECK(radar.giveCon(RadarVo) && radar.giveAge(RadarVo) == 10);
        radar.markRemoteLost();
        CHECK(!radar.giveCon(RadarVo) && !radar.giveCon(RadarRe));
        CHECK(radar[RadarVo].DistRaw == 1010 && radar.giveAge(RadarVo) == 10); // Werte bleiben stehen und altern
        g_us = 6000000;
        CHECK(radar.giveAge(RadarVo) == 1010);
    }
    printf("\n%d Pruefungen, %d Fehler\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
