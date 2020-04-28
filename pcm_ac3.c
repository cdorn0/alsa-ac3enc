/*
 *  This file is part of alsa-ac3enc.
 *
 *  Copyright (c) 2020 Christian Dorn <christian.dorn@tng-project.de>
 *
 *  alsa-ac3enc is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  alsa-ac3enc is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with alsa-ac3enc.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <string.h>

#define __USE_XOPEN
#include <unistd.h>

#include <alsa/asoundlib.h>
#include <alsa/pcm_external.h>

#include <aften/aften.h>
#include <aften/aften-types.h>

#define IEC958_HEADER_SIZE  8
#define IEC958_FRAME_SIZE   6144

static void ac3enc_fillIEC958Header(unsigned char *buf, int ac3outsize) {
#if __BYTE_ORDER == __BIG_ENDIAN
    buf[0] = 0xf8;	/* BEGIN SYNCWORD */
    buf[1] = 0x72;
    buf[2] = 0x4e;
    buf[3] = 0x1f; /* END SYNCWORD */
    buf[4] = buf[13] & 7; /* AC3 DATA */
    buf[5] = 0x01; /* BSMOD, STREAM = 0 */
    buf[6] = (ac3outsize >> 5) & 0xff;
    buf[7] = (ac3outsize << 3) & 0xff;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
    buf[0] = 0x72;	/* BEGIN SYNCWORD */
    buf[1] = 0xf8;
    buf[2] = 0x1f;
    buf[3] = 0x4e;  /* END SYNCWORD */
    buf[4] = 0x01;  /* AC3 DATA */
    buf[5] = buf[13] & 7; /* BSMOD, STREAM = 0 */
    buf[6] = (ac3outsize << 3) & 0xff;
    buf[7] = (ac3outsize >> 5) & 0xff;

    /* Swap bytes of the encoded frame */
    swab(buf + IEC958_HEADER_SIZE, buf + IEC958_HEADER_SIZE, ac3outsize);
#endif
    /* Fill remaining bytes of the IEC958 Frame with zeros */
    memset(buf + IEC958_HEADER_SIZE + ac3outsize, 0x00, IEC958_FRAME_SIZE - ac3outsize - IEC958_HEADER_SIZE);
}

AftenContext *ac3enc_create(int threads, int sample_rate, unsigned int channels) {
    AftenContext* afctx = calloc(1, sizeof(AftenContext));

    aften_set_defaults(afctx);

    afctx->samplerate = sample_rate;
    afctx->sample_format = A52_SAMPLE_FMT_FLT;

    switch(channels) {
        case 1:
            afctx->acmod = A52_ACMOD_MONO;
            afctx->lfe = 0;
            afctx->channels = 1;
            break;
        case 2:
            afctx->acmod = A52_ACMOD_STEREO;
            afctx->lfe = 0;
            afctx->channels = 2;
            break;
        case 3:
            afctx->acmod = A52_ACMOD_STEREO;
            afctx->lfe = 1;
            afctx->channels = 3;
            break;
        case 4:
            afctx->acmod = A52_ACMOD_2_2;
            afctx->lfe = 0;
            afctx->channels = 4;
            break;
        case 5:
            afctx->acmod = A52_ACMOD_3_2;
            afctx->lfe = 0;
            afctx->channels = 5;
            break;
        case 6:
            afctx->acmod = A52_ACMOD_3_2;
            afctx->lfe = 1;
            afctx->channels = 6;
            break;
        default:
            return NULL;
    }

    afctx->mode = AFTEN_ENCODE;
    afctx->verbose = 0;
    afctx->system.n_threads = threads;

    int err = aften_encode_init(afctx);

    if (err != 0) {
        return NULL;
    }

    return afctx;
}

inline int ac3enc_convert(AftenContext* c, const float *input, uint8_t *output) {
    return aften_encode_frame(c, (unsigned char *) output, input, A52_SAMPLES_PER_FRAME);
}

int ac3enc_destroy(AftenContext* c) {
    if (!c) {
        return -EINVAL;
    }

    free(c);

    return 0;
}

struct pcm_buffer {
    float* buffer;
    snd_pcm_uframes_t size;
    snd_pcm_uframes_t index;
};

static inline void ac3plug_pcm_buffer_rewind(struct pcm_buffer* pcmbuf, snd_pcm_uframes_t count) {
    int idx = (pcmbuf->index - count) % pcmbuf->size;

    if (idx < 0)
        pcmbuf->index = pcmbuf->size + idx;
    else
        pcmbuf->index = idx;
}

static inline void ac3plug_pcm_buffer_push(struct pcm_buffer* pcmbuf, float value) {
    pcmbuf->buffer[pcmbuf->index++] = value;
    /* Wrap around buffer size */
    pcmbuf->index %= pcmbuf->size;
}

static void ac3plug_pcm_buffer_get(struct pcm_buffer* pcmbuf, float* output, snd_pcm_uframes_t size) {
    int offset = pcmbuf->index - size;

    /* Wrap around buffer size */
    if (offset < 0) {
        memcpy(output, &pcmbuf->buffer[pcmbuf->size + offset] , (-offset)*sizeof(float));
        memcpy(output-offset, &pcmbuf->buffer[0], (size+offset)*sizeof(float));
    } else {
        memcpy(output, &pcmbuf->buffer[offset], size*sizeof(float));
    }
}

struct ac3plug_info {
    snd_pcm_extplug_t ext;

    snd_pcm_uframes_t pcm_buffer_max_size;

    AftenContext* afctx;
    unsigned int threads;

    struct pcm_buffer pcmbuf;
    int16_t iec958_buffer[IEC958_FRAME_SIZE];
    snd_pcm_uframes_t iec958_bufpos;

    snd_pcm_uframes_t src_next_offset;
};

static inline void *area_addr(const snd_pcm_channel_area_t *area, snd_pcm_uframes_t offset)
{
    unsigned int bitofs = area->first + area->step * offset;
    return (char *) area->addr + bitofs / 8;
}

static inline float get_float(const snd_pcm_channel_area_t *area, snd_pcm_uframes_t offset, snd_pcm_format_t format)
{
    switch (format) {
        case SND_PCM_FORMAT_S16: {
            int16_t sample16 = *((int16_t*)area_addr(area, offset));
            return (float) (sample16) / 32768.0f;
        }
        case SND_PCM_FORMAT_S32: {
            int32_t sample32 = *((int32_t*) area_addr(area, offset));
            return sample32 / 2147483648.0f;
        }
        case SND_PCM_FORMAT_FLOAT: {
            float samplef = *((float*)area_addr(area, offset));
            return samplef;
        }

        default:
            break;
    }

    return 0;
}

float inbuffer[6 * A52_SAMPLES_PER_FRAME];
static inline void put_s16(const snd_pcm_channel_area_t *area, snd_pcm_uframes_t offset, int16_t value)
{
    int16_t* sample16 = (int16_t*)area_addr(area, offset);
    *sample16 = value;
}

static snd_pcm_sframes_t ac3plug_transfer(snd_pcm_extplug_t *ext,
                                          const snd_pcm_channel_area_t *dst_areas, snd_pcm_uframes_t dst_offset,
                                          const snd_pcm_channel_area_t *src_areas, snd_pcm_uframes_t src_offset,
                                          snd_pcm_uframes_t size)
{
    struct ac3plug_info *ac3plug = (struct ac3plug_info*) ext;

    /* Try to detect a rewind of the buffer (LIMIT: If rewound to zero, it is not detected) */
    if ((src_offset > 0) && (src_offset < ac3plug->src_next_offset)) {
        snd_pcm_uframes_t difference = ac3plug->src_next_offset - src_offset;
        ac3plug_pcm_buffer_rewind(&ac3plug->pcmbuf, ext->channels*difference);

        /* If rewound drop recent IEC958 frame and convert new one. This is not an optimal solution as frames are dropped (TODO: is it allowed in spec?) */
        ac3plug->iec958_bufpos = 0;
    }

    /* Output Buffer must not be overfilled, limit count of the samples copied to A52_SAMPLES_PER_FRAME
     * TODO: To save computational power wrap in while and encode as chunks
     * */
    snd_pcm_uframes_t remaining = A52_SAMPLES_PER_FRAME - ac3plug->iec958_bufpos;
    if (size > remaining)
        size = remaining;

    /* Save the the position where the next transfer should take place */
    ac3plug->src_next_offset = src_offset + size;

    /* If the buffer position is zero, encode the last A52_SAMPLES_PER_FRAME samples in the PCM buffer */
    if (ac3plug->iec958_bufpos == 0) {
        ac3plug_pcm_buffer_get(&ac3plug->pcmbuf, inbuffer, ext->channels*A52_SAMPLES_PER_FRAME);

        int outsize = ac3enc_convert(ac3plug->afctx, inbuffer, (uint8_t *) ac3plug->iec958_buffer + IEC958_HEADER_SIZE);
        if (outsize < 0) {
            SNDERR("AC3 conversion failed");

            return -EIO;
        }

        ac3enc_fillIEC958Header((uint8_t *) ac3plug->iec958_buffer, outsize);
    }

    int dstbufidx = 2 * ac3plug->iec958_bufpos;

    for (snd_pcm_uframes_t i = 0; i < size; i++) {
        /* Read samples to pcm buffer according to channel count*/
        switch (ext->channels) {
            case 1:
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[0], i + src_offset, ext->format));
                break;
            case 2:
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[0], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[1], i + src_offset, ext->format));
                break;
            case 3:
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[0], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[1], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[2], i + src_offset, ext->format));
                break;
            case 4:
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[0], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[1], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[2], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[3], i + src_offset, ext->format));
                break;
            case 5:
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[0], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[4], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[1], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[2], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[3], i + src_offset, ext->format));
                break;
            case 6:
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[0], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[4], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[1], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[2], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[3], i + src_offset, ext->format));
                ac3plug_pcm_buffer_push(&ac3plug->pcmbuf, get_float(&src_areas[5], i + src_offset, ext->format));
                break;

            default:
                return -EINVAL;
        }

        /* Write out encoded samples to IEC958 (int16 stereo) */
        put_s16(&dst_areas[0], i + dst_offset, ac3plug->iec958_buffer[dstbufidx++]);
        put_s16(&dst_areas[1], i + dst_offset, ac3plug->iec958_buffer[dstbufidx++]);
    }

    ac3plug->iec958_bufpos += size;

    if (ac3plug->iec958_bufpos == A52_SAMPLES_PER_FRAME) {
        ac3plug->iec958_bufpos = 0;
    }

    return size;
}

static int ac3plug_init(snd_pcm_extplug_t *ext)
{
    struct ac3plug_info *ac3plug = (struct ac3plug_info*) ext;

    if (ext->rate != 32000 && ext->rate != 44100 && ext->rate != 48000) {
        SNDERR("Wrong sample rate, must be 32000, 44100 or 48000 Hz");
        return -EINVAL;
    }

    if (ext->channels < 1 || ext->channels > 6) {
        SNDERR("Wrong number of channels");
        return -EINVAL;
    }

    ac3plug->pcmbuf.index = 0;
    ac3plug->pcmbuf.size = ac3plug->pcm_buffer_max_size;
    ac3plug->pcmbuf.buffer = calloc(ac3plug->pcmbuf.size * 6, sizeof(float));

    if (!ac3plug->pcmbuf.buffer) {
        SNDERR("Error allocating memory for PCM buffer");
        return -ENOMEM;
    }

    memset(ac3plug->pcmbuf.buffer, 0x00, ac3plug->pcmbuf.size);

    ac3plug->afctx = ac3enc_create(ac3plug->threads, ext->rate, ext->channels);

    if (!ac3plug->afctx) {
        SNDERR("Failed to create AC3 encoder");
        return -ENOMEM;
    }

    ac3plug->src_next_offset = 0;
    ac3plug->iec958_bufpos = 0;


    return 0;
}

static int ac3plug_close(snd_pcm_extplug_t *ext)
{
    struct ac3plug_info *ac3plug = (struct ac3plug_info*)ext;

    ac3enc_destroy(ac3plug->afctx);
    ac3plug->afctx = NULL;
    return 0;
}

static unsigned int chmap[6][6] = {
        { SND_CHMAP_MONO }, /* Mono */
        { SND_CHMAP_FL, SND_CHMAP_FR }, /* Stereo */
        { SND_CHMAP_FL, SND_CHMAP_FR, SND_CHMAP_LFE }, /* 2.1 */
        { SND_CHMAP_FL, SND_CHMAP_FR, SND_CHMAP_RL, SND_CHMAP_RR }, /* 4.0 */
        { SND_CHMAP_FL, SND_CHMAP_FR, SND_CHMAP_RL, SND_CHMAP_RR, SND_CHMAP_FC}, /* 5.0 */
        { SND_CHMAP_FL, SND_CHMAP_FR, SND_CHMAP_RL, SND_CHMAP_RR, SND_CHMAP_FC, SND_CHMAP_LFE} /* 5.1 */
};

static snd_pcm_chmap_query_t **ac3plug_query_chmaps(snd_pcm_extplug_t *ext ATTRIBUTE_UNUSED)
{
    snd_pcm_chmap_query_t **maps;
    int i;

    maps = calloc(7, sizeof(void *));
    if (!maps)
        return NULL;

    for (i = 0; i < 6; i++) {
        snd_pcm_chmap_query_t *p;
        p = maps[i] = calloc((i + 1) + 2, sizeof(int));

        if (!p) {
            snd_pcm_free_chmaps(maps);
            return NULL;
        }

        p->type = SND_CHMAP_TYPE_FIXED;
        p->map.channels = i + 1;
        memcpy(p->map.pos, &chmap[i][0], (i + 1) * sizeof(int));
    }

    return maps;
}

static snd_pcm_chmap_t *ac3plug_get_chmap(snd_pcm_extplug_t *ext)
{
    snd_pcm_chmap_t *map;

    if (ext->channels < 1 || ext->channels > 6)
        return NULL;
    map = malloc((ext->channels + 1) * sizeof(int));
    if (!map)
        return NULL;

    map->channels = ext->channels;
    memcpy(map->pos, &chmap[ext->channels - 1][0], ext->channels * sizeof(int));

    return map;
}

static const snd_pcm_extplug_callback_t ac3plug_callback = {
        .transfer = ac3plug_transfer,
        .init = ac3plug_init,
        .close = ac3plug_close,
        .query_chmaps = ac3plug_query_chmaps,
        .get_chmap = ac3plug_get_chmap,
};

SND_PCM_PLUGIN_DEFINE_FUNC(ac3)
{
    snd_config_iterator_t i, next;
    snd_config_t *slave = NULL;

    unsigned int threads = 2;

    struct ac3plug_info *ac3plug;
    int err;

    if (stream != SND_PCM_STREAM_PLAYBACK) {
        SNDERR("AC3 is only for playback");
        return -EINVAL;
    }

    snd_config_for_each(i, next, conf) {
        snd_config_t *n = snd_config_iterator_entry(i);
        const char *id;
        if (snd_config_get_id(n, &id) < 0)
            continue;
        if (strcmp(id, "comment") == 0 || strcmp(id, "type") == 0 || strcmp(id, "hint") == 0)
            continue;
        if (strcmp(id, "slave") == 0) {
            slave = n;
            continue;
        }
        if (strcmp(id, "threads") == 0) {
            long val;
            if (snd_config_get_integer(n, &val) < 0) {
                SNDERR("Invalid type for %s", id);
                return -EINVAL;
            }

            if (val < 0 || val > 32) {
                SNDERR("threads must be between 0 and 32");
                return -EINVAL;
            }

            threads = (unsigned int) val;

            continue;
        }

        SNDERR("Unknown field %s", id);
        return -EINVAL;
    }

    if (!slave) {
        SNDERR("No slave defined for ac3");
        return -EINVAL;
    }

    ac3plug = calloc(1, sizeof(*ac3plug));
    if (ac3plug == NULL)
        return -ENOMEM;

    ac3plug->threads = threads;

    ac3plug->ext.version = SND_PCM_EXTPLUG_VERSION;
    ac3plug->ext.name = "AC3 encoder";
    ac3plug->ext.callback = &ac3plug_callback;
    ac3plug->ext.private_data = ac3plug;

    err = snd_pcm_extplug_create(&ac3plug->ext, name, root, slave, stream, mode);
    if (err < 0) {
        ac3enc_destroy(ac3plug->afctx);
        free(ac3plug);
        return err;
    }

    static const unsigned int channels[6] = {1, 2, 3, 4, 5, 6};
    static const unsigned int formats[3] = {SND_PCM_FORMAT_FLOAT, SND_PCM_FORMAT_S32, SND_PCM_FORMAT_S16};

    snd_pcm_extplug_set_param_list(&ac3plug->ext, SND_PCM_EXTPLUG_HW_CHANNELS, 6, channels);
    snd_pcm_extplug_set_param_list(&ac3plug->ext, SND_PCM_EXTPLUG_HW_FORMAT, 3, formats);

    snd_pcm_extplug_set_slave_param(&ac3plug->ext, SND_PCM_EXTPLUG_HW_CHANNELS, 2);
    snd_pcm_extplug_set_slave_param(&ac3plug->ext, SND_PCM_EXTPLUG_HW_FORMAT, SND_PCM_FORMAT_S16);

    *pcmp = ac3plug->ext.pcm;

    snd_pcm_hw_params_t* params;

    if ((err = snd_pcm_hw_params_malloc(&params)) < 0)
        return err;

    if ((err = snd_pcm_hw_params_any(ac3plug->ext.pcm, params)) < 0) {
        SNDERR("Cannot get slave hw_params");
        free (params);

        return err;
    }

    snd_pcm_uframes_t buffer_max;
    if ((err = snd_pcm_hw_params_get_buffer_size_max(params, &buffer_max)) < 0) {
        SNDERR("Cannot get max buffer_size");
        free (params);

        return err;
    }

    ac3plug->pcm_buffer_max_size = buffer_max;

    return 0;
}

SND_PCM_PLUGIN_SYMBOL(ac3);