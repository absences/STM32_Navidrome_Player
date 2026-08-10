#ifndef WM8978_PLAYBACK_H
#define WM8978_PLAYBACK_H

#include <stdint.h>

uint32_t WM8978_PlaybackInit(void);
void WM8978_SetSampleRate(uint32_t sample_rate);
void WM8978_SetVolume(uint8_t volume);
void WM8978_SetPlaying(uint32_t playing);
uint32_t WM8978_IsPlaying(void);
uint8_t WM8978_GetVolume(void);
uint32_t WM8978_QueuePCM(const int16_t *samples, uint32_t frames,
                         uint32_t channels);
uint32_t WM8978_BufferedFrames(void);
uint32_t WM8978_PlayedSeconds(void);
uint32_t WM8978_HasPlayedAudio(void);
void WM8978_ClearPCM(void);
void WM8978_ResetPlayedTime(void);

#endif
