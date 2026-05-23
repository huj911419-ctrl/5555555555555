#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 0 = debug/tuning mode, 1 = race mode. */
#ifndef RACE_MODE
#define RACE_MODE 0
#endif

/* Otsu threshold update interval. Larger values reduce CPU cost. */
#define TF_OTSU_INTERVAL_DEBUG 2u
#define TF_OTSU_INTERVAL_RACE  4u

/* Optional fastest mode for stable lighting: 1 uses fixed threshold only. */
#define TF_FIXED_THRESHOLD_ENABLE 0
#define TF_FIXED_THRESHOLD_VALUE  75

#endif /* APP_CONFIG_H */
