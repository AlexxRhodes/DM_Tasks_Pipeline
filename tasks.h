#pragma once

#include <stdint.h>

#define N_BODIES 9
#define G 3e-4

// Types
struct Body {
    double x;
    double y;
    double vx;
    double vy;
    double mass;
    double radius;
    double radius_scale;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct Image {
  uint8_t *data;
  int width;
  int height;
};

struct ImageStats {
  uint8_t min;
  uint8_t max;
  uint8_t mode;
  double mean;
  double median;
};

enum Step {
  NBODIES_SIMULATION
, IMAGE_GENERATION
, IMAGE_GAUSSIAN_BLUR
, IMAGE_SAVE_FS
, IMAGE_GRAYSCALE
, IMAGE_STATS
, STATS_SAVE_FS
, STEP_MAX
, STEP_MIN = NBODIES_SIMULATION
};

// Functions related to basic image manipulation.
void set_img_blank(struct Image * img);
struct Image * alloc_img(int width, int height);
void free_img(struct Image * img);

// Functions related to time measurement and stats.
int64_t ns_diff(const struct timespec *t0, const struct timespec *t1);
void print_duration(const char * prefix, int64_t ns, int64_t total_ns);
void print_elapsed_time_stats(int64_t total_ns);

// Functions that implement tasks.
void simulate_n_bodies(struct Body bodies[], int n, double dt);
void generate_image_from_bodies(struct Body bodies[], int n, struct Image * img);
void apply_gaussian_blur(struct Image *img_in, struct Image *img_out);
void convert_to_grayscale(struct Image *img_in, struct Image *img_out);
void compute_image_statistics(const struct Image *img, struct ImageStats *stats);
void save_stats(const struct ImageStats *stats, const char *filename, int current_step);
void save_img_as_png(const struct Image *img, const char *filename_format, int current_step);
