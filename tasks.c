#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <unistd.h>
#include <sys/stat.h>

#include <png.h>

#include "tasks.h"

// global variables
const double x_min = -60;
const double x_max = 80;
const double y_min = -30;
const double y_max = 30;

int64_t cum_ns[STEP_MAX] = {0};
const char * step_cstr[STEP_MAX] = {
  "nbodies_simulation"
, "image_generation"
, "image_gaussian_blur"
, "image_save_fs"
, "image_grayscale"
, "image_stats"
, "stats_save_fs"
};

void set_img_blank(struct Image * img) {
  memset(img->data, 0, 3 * img->width * img->height);
}

struct Image * alloc_img(int width, int height) {
  struct Image * img = malloc(sizeof(struct Image));
  img->width = width;
  img->height = height;
  img->data = (uint8_t *)malloc(3 * img->width * img->height * sizeof(uint8_t));
  set_img_blank(img);
  return img;
}

void free_img(struct Image * img) {
  free(img->data);
  img->data = NULL;
  free(img);
}

int64_t ns_diff(const struct timespec *t0, const struct timespec *t1) {
  int64_t s_diff = t1->tv_sec - t0->tv_sec;
  int64_t ns_diff = t1->tv_nsec - t0->tv_nsec;
  return s_diff * 1000000000LL + ns_diff;
}

void print_duration(const char * prefix, int64_t ns, int64_t total_ns) {
  double proportion = 100.0 * ns / total_ns;
  int64_t seconds = ns / 1000000000L;
  ns %= 1000000000L;

  int64_t milliseconds = ns / 1000000L;
  ns %= 1000000L;

  if (seconds > 0) {
    printf("  %19s: %3ld s, %3ld ms, %6ld ns (%6.2f %% du total)\n", prefix, seconds, milliseconds, ns, proportion);
  } else if (milliseconds > 0) {
    printf("  %19s:        %3ld ms, %6ld ns (%6.2f %% du total)\n", prefix, milliseconds, ns, proportion);
  } else {
    printf("  %19s:                %6ld ns (%6.2f %% du total)\n", prefix, ns, proportion);
  }
}

void print_elapsed_time_stats(int64_t total_ns) {
  print_duration("temps total", total_ns, total_ns);
  printf("\ndétail par étape\n");
  for (int step = STEP_MIN; step < STEP_MAX; ++step) {
    print_duration(step_cstr[step], cum_ns[step], total_ns);
  }
}

void simulate_n_bodies(struct Body bodies[], int n, double dt) {
  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  for (int i = 0; i < n; i++) {
    double ax = 0;
    double ay = 0;

    for (int j = 0; j < n; j++) {
      if (i != j) {
        double dx = bodies[j].x - bodies[i].x;
        double dy = bodies[j].y - bodies[i].y;
        double distance_squared = dx * dx + dy * dy;
        double distance = sqrt(distance_squared);
        double force = (G * bodies[i].mass * bodies[j].mass) / distance_squared;
        ax += force * dx / (distance * bodies[i].mass);
        ay += force * dy / (distance * bodies[i].mass);
      }
    }

    bodies[i].vx += ax * dt;
    bodies[i].vy += ay * dt;
  }

  for (int i = 0; i < n; i++) {
    bodies[i].x += bodies[i].vx * dt;
    bodies[i].y += bodies[i].vy * dt;
  }

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }
  cum_ns[NBODIES_SIMULATION] += ns_diff(&t0, &t1);
}

void generate_image_from_bodies(struct Body bodies[], int n, struct Image * img) {
  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  set_img_blank(img);

  for (int i = 0; i < n; i++) {
    int x = (bodies[i].x - x_min) / (x_max - x_min) * img->width;
    int y = (bodies[i].y - y_min) / (y_max - y_min) * img->height;

    int r = bodies[i].radius * bodies[i].radius_scale / (x_max - x_min) * img->width;
    int r_squared = r * r;

    for (int dx = -r; dx <= r; dx++) {
      for (int dy = -r; dy <= r; dy++) {
        int nx = x + dx;
        int ny = y + dy;
        if (nx >= 0 && nx < img->width && ny >= 0 && ny < img->height) {
          int dist_squared = dx * dx + dy * dy;
          if (dist_squared <= r_squared) {
            int idx = 3 * (ny * img->width + nx);
            img->data[idx + 0] = bodies[i].r;
            img->data[idx + 1] = bodies[i].g;
            img->data[idx + 2] = bodies[i].b;
          }
        }
      }
    }
  }

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }
  cum_ns[IMAGE_GENERATION] += ns_diff(&t0, &t1);
}

void apply_gaussian_blur(struct Image *img_in, struct Image *img_out) {
  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  const int kernel_size = 5;
  const double sigma = 1.0;
  double kernel[kernel_size][kernel_size];
  int half_size = kernel_size / 2;

  double sum = 0.0;
  for (int i = 0; i < kernel_size; i++) {
    for (int j = 0; j < kernel_size; j++) {
      int x = i - half_size;
      int y = j - half_size;
      kernel[i][j] = exp(-(x * x + y * y) / (2 * sigma * sigma));
      sum += kernel[i][j];
    }
  }

  for (int i = 0; i < kernel_size; i++) {
    for (int j = 0; j < kernel_size; j++) {
      kernel[i][j] /= sum;
    }
  }

  set_img_blank(img_out);

  for (int y = 0; y < img_in->height; y++) {
    for (int x = 0; x < img_in->width; x++) {
      double r = 0, g = 0, b = 0;
      for (int ky = 0; ky < kernel_size; ky++) {
        for (int kx = 0; kx < kernel_size; kx++) {
          int ny = y + ky - half_size;
          int nx = x + kx - half_size;
          if (nx >= 0 && nx < img_in->width && ny >= 0 && ny < img_in->height) {
            int idx = 3 * (ny * img_in->width + nx);
            r += img_in->data[idx] * kernel[ky][kx];
            g += img_in->data[idx + 1] * kernel[ky][kx];
            b += img_in->data[idx + 2] * kernel[ky][kx];
          }
        }
      }
      int idx = 3 * (y * img_in->width + x);
      img_out->data[idx] = (uint8_t)r;
      img_out->data[idx + 1] = (uint8_t)g;
      img_out->data[idx + 2] = (uint8_t)b;
    }
  }

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }
  cum_ns[IMAGE_GAUSSIAN_BLUR] += ns_diff(&t0, &t1);
}

void convert_to_grayscale(struct Image *img_in, struct Image *img_out) {
  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  for (int i = 0; i < img_in->width * img_in->height; i++) {
    int idx = 3 * i;
    uint8_t gray = (uint8_t)(0.299 * img_in->data[idx] + 0.587 * img_in->data[idx + 1] + 0.114 * img_in->data[idx + 2]);
    img_out->data[idx] = img_out->data[idx + 1] = img_out->data[idx + 2] = gray;
  }

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }
  cum_ns[IMAGE_GRAYSCALE] += ns_diff(&t0, &t1);
}

void compute_image_statistics(const struct Image *img, struct ImageStats *stats) {
  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  stats->min = 255;
  stats->max = 0;
  stats->mode = 0;

  int histogram[256] = {0};
  double sum = 0;

  for (int i = 0; i < img->width * img->height; i++) {
    int idx = 3 * i;
    uint8_t gray = img->data[idx];

    sum += gray;
    histogram[gray]++;
    if (gray < stats->min) stats->min = gray;
    if (gray > stats->max) stats->max = gray;
  }
  
  int median1 = -1, median2 = -1;
  int max_count = 0;
  int cumulative_count = 0;
  int total_count = img->width * img->height;
  int mid1 = total_count / 2 - 1;
  int mid2 = total_count / 2;

  for (int i = 0; i < 256; ++i) {
    if (histogram[i] > max_count) {
      max_count = histogram[i];
      stats->mode = i;
    }

    cumulative_count += histogram[i];
    if (median1 == -1 && cumulative_count > mid1) {
      median1 = i;
    }
    if (median2 == -1 && cumulative_count > mid2) {
      median2 = i;
    }
  }

  stats->mean = sum / (total_count);
  stats->median = (median1 + median2) / 2.0;

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }
  cum_ns[IMAGE_STATS] += ns_diff(&t0, &t1);
}

void save_stats(const struct ImageStats *stats, const char *filename, int current_step) {
  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  struct stat buffer;
  bool exists = (stat(filename, &buffer) == 0);

  FILE *file = fopen(filename, "a");

  if (file == NULL) {
    perror("cannot save stats to file");
    exit(1);
  }

  if (!exists) {
    fprintf(file, "step,min,max,mode,mean,median\n");
  }

  fprintf(file, "%d,%d,%d,%d,%.2f,%.2f\n", current_step, stats->min, stats->max, stats->mode, stats->mean, stats->median);
  fclose(file);

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }
  cum_ns[STATS_SAVE_FS] += ns_diff(&t0, &t1);
}

void save_img_as_png(const struct Image *img, const char *filename_format, int current_step) {
  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  char filename[256];
  snprintf(filename, 256, filename_format, current_step);

  FILE *fp = fopen(filename, "wb");
  if (!fp) {
    fprintf(stderr, "cannot open file '%s': %s\n", filename, strerror(errno));
    return;
  }

  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png) return;

  png_infop info = png_create_info_struct(png);
  if (!info) return;

  if (setjmp(png_jmpbuf(png))) return;

  png_init_io(png, fp);

  png_set_IHDR(
    png,
    info,
    img->width, img->height,
    8,
    PNG_COLOR_TYPE_RGB,
    PNG_INTERLACE_NONE,
    PNG_COMPRESSION_TYPE_DEFAULT,
    PNG_FILTER_TYPE_DEFAULT
  );

  png_write_info(png, info);

  png_bytep row = (png_bytep)malloc(3 * img->width * sizeof(png_byte));
  for (int y = 0; y < img->height; y++) {
    memcpy(row, &img->data[y * img->width * 3], 3 * img->width);
    png_write_row(png, row);
  }

  png_write_end(png, NULL);
  fclose(fp);
  png_destroy_write_struct(&png, &info);
  free(row);

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }
  cum_ns[IMAGE_SAVE_FS] += ns_diff(&t0, &t1);
}
