#ifndef WAVEFORM_H
#define WAVEFORM_H

// Format: {Number of harmonics, harmonic weights....} - 1st element MUST indicate the number of harmonics (i.e. length of the array - 1)
const float tpt_basic[9] = {8, 1.0, 0.6, 0.45, 0.3, 0.25, 0.15, 0.1, 0.07};
const float tpt_enriched[13] = {12, 1.0, 0.85, 0.75, 0.65, 0.5, 0.35, 0.2, 0.12, 0.07, 0.04, 0.02, 0.01};
const float fusionSynthHarmonics[13] = {12, 1.0, 0.8, 0.6, 0.5, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.08, 0.05};
const float chickCoreaHarmonics[9] = {8, 1.0, 0.3, 0.7, 0.4, 0.6, 0.3, 0.1, 0.05};
const float woodwindHarmonics[10] = {9, 1.0, 0.1, 0.7, 0.05, 0.5, 0.2, 0.3, 0.1, 0.05};
const float flugelHarmonics[11] = {10, 1.0, 0.75, 0.55, 0.35, 0.22, 0.12, 0.06, 0.03, 0.015, 0.008};
const float fluteHarmonics[8] = {7, 1.0, 0.12, 0.05, 0.20, 0.03, 0.10, 0.02};
const float guitarHarmonics[11] = {10, 1.0, 0.65, 0.50, 0.35, 0.25, 0.18, 0.12, 0.08, 0.05, 0.03};
const float saxHarmonics[13] = {12, 1.0, 0.55, 0.90, 0.50, 0.75, 0.40, 0.60, 0.30, 0.45, 0.25, 0.35, 0.18};
const float violinHarmonics[12] = {11, 1.0, 0.8, 0.65, 0.55, 0.45, 0.38, 0.3, 0.22, 0.17, 0.13, 0.1};


#endif