#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "tasks.h"

int main(int argc, char *argv[]) {
  if (argc < 1)
    exit(1);
  if (argc != 5) {
    fprintf(stderr, "usage: %s <nb-steps> <img-width> <img-height> <save-img>\n", argv[0]);
    exit(1);
  }

  int nb_steps = atoi(argv[1]);
  int width = atoi(argv[2]);
  int height = atoi(argv[3]);
  int save_img = atoi(argv[4]);

  struct Body bodies[N_BODIES] = {
    { 0.00, 0.0,  0.000, 0.0,      1.0, 0.00465047,  5.0e2,  255, 204,   0},
    { 0.39, 0.0,  0.323, 0.0,  1.65e-7,   1.765e-5, 10.0e3,  169, 169, 169},
    { 0.72, 0.0,  0.218, 0.0,  2.45e-6,   4.552e-5, 10.0e3,  255, 204, 153},
    { 1.00, 0.0,  0.170, 0.0,  3.00e-6,   4.258e-5, 10.0e3,    0, 102, 204},
    { 1.52, 0.0,  0.128, 0.0,  3.21e-7,   2.279e-5, 10.0e3,  255, 102,   0},
    { 5.20, 0.0,  0.060, 0.0,  9.55e-4,  4.7789e-4,  5.0e3,   204, 153, 102},
    { 9.58, 0.0,  0.043, 0.0,  2.86e-4,  4.0072e-4,  5.0e3,   210, 180, 140},
    {19.22, 0.0,  0.030, 0.0,  4.36e-5,  1.6938e-4,  5.0e3,   173, 216, 230},
    {30.05, 0.0,  0.024, 0.0,  5.17e-5,  1.6418e-4,  5.0e3,     0,   0, 128},
  };

  srand(1);
  for (int i = 1; i < N_BODIES; ++i) {
    double dist = bodies[i].x;
    double angle = 2 * M_PI * (double)rand() / RAND_MAX;
    bodies[i].x = dist * cos(angle);
    bodies[i].y = dist * sin(angle);

    double rot_speed = bodies[i].vx;
    bodies[i].vx = -rot_speed * bodies[i].y;
    bodies[i].vy =  rot_speed * bodies[i].x;
  }

  const char * stats_filename = "./img-stats.csv";
  const char * png_filename_format = "./img%03d.png";

  // clean files
  remove(stats_filename);
  char filename[256];
  if (save_img) {
    for (int i = 0; i < nb_steps; ++i) {
      snprintf(filename, 256, png_filename_format, i);
      remove(filename);
    }
  }

  struct Image * img1 = alloc_img(width, height);
  struct Image * img2 = alloc_img(width, height);
  struct ImageStats stats;

  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  for (int current_step = 0; current_step < nb_steps; ++current_step) {
    simulate_n_bodies(bodies, N_BODIES, 1.0);

    generate_image_from_bodies(bodies, N_BODIES, img1);
    apply_gaussian_blur(img1, img2);

    if (save_img)
      save_img_as_png(img2, png_filename_format, current_step);

    convert_to_grayscale(img2, img1);
    compute_image_statistics(img1, &stats);
    save_stats(&stats, stats_filename, current_step);
  }

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1) {
    perror("clock_gettime");
    exit(1);
  }

  int64_t total_ns = ns_diff(&t0, &t1);
  print_elapsed_time_stats(total_ns);

  free_img(img1); img1 = NULL;
  free_img(img2); img2 = NULL;

  return 0;
}
