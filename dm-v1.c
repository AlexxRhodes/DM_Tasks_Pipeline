#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>

#include "tasks.h"

#define NUM_THREADS 5 // Nombre de threads pour chaque type de tâche (sans sauvegarde d'image)

// Déclaration globale et des semaphores
int nb_steps, width, height, save_img;
sem_t sem_img_gen, sem_img_blur, sem_img_gray, sem_img_stats, sem_img_save;
struct Image *img1, *img2;
struct ImageStats stats;
const char *stats_filename = "./img-stats.csv";
const char *png_filename_format = "./img%03d.png";
struct Body bodies[N_BODIES] = {
    {0.00, 0.0, 0.000, 0.0, 1.0, 0.00465047, 5.0e2, 255, 204, 0},
    {0.39, 0.0, 0.323, 0.0, 1.65e-7, 1.765e-5, 10.0e3, 169, 169, 169},
    {0.72, 0.0, 0.218, 0.0, 2.45e-6, 4.552e-5, 10.0e3, 255, 204, 153},
    {1.00, 0.0, 0.170, 0.0, 3.00e-6, 4.258e-5, 10.0e3, 0, 102, 204},
    {1.52, 0.0, 0.128, 0.0, 3.21e-7, 2.279e-5, 10.0e3, 255, 102, 0},
    {5.20, 0.0, 0.060, 0.0, 9.55e-4, 4.7789e-4, 5.0e3, 204, 153, 102},
    {9.58, 0.0, 0.043, 0.0, 2.86e-4, 4.0072e-4, 5.0e3, 210, 180, 140},
    {19.22, 0.0, 0.030, 0.0, 4.36e-5, 1.6938e-4, 5.0e3, 173, 216, 230},
    {30.05, 0.0, 0.024, 0.0, 5.17e-5, 1.6418e-4, 5.0e3, 0, 0, 128},
};

void *f_simu(void *arg)
{
  for (int i = 0; i < nb_steps; i++)
  {
    simulate_n_bodies(bodies, N_BODIES, 1.0);
    sem_post(&sem_img_gen);
  }
  return NULL;
}

void *f_img_gen(void *arg)
{
  for (int i = 0; i < nb_steps; i++)
  {
    sem_wait(&sem_img_gen);
    generate_image_from_bodies(bodies, N_BODIES, img1);
    sem_post(&sem_img_blur);
  }
  return NULL;
}

void *f_img_blur(void *arg)
{
  for (int i = 0; i < nb_steps; i++)
  {
    sem_wait(&sem_img_blur);
    apply_gaussian_blur(img1, img2);
    sem_post(&sem_img_gray);
  }
  return NULL;
}

void *f_img_gray(void *arg)
{
  for (int i = 0; i < nb_steps; i++)
  {
    sem_wait(&sem_img_gray);
    convert_to_grayscale(img2, img1);
    sem_post(&sem_img_stats);
  }
  return NULL;
}

void *f_img_stats(void *arg)
{
  for (int i = 0; i < nb_steps; i++)
  {
    sem_wait(&sem_img_stats);
    compute_image_statistics(img1, &stats);
    save_stats(&stats, stats_filename, i);
    if (save_img)
      sem_post(&sem_img_save);
  }
  return NULL;
}

void *f_img_save(void *arg)
{
  if (!save_img)
    return NULL;
  for (int i = 0; i < nb_steps; i++)
  {
    sem_wait(&sem_img_save);
    save_img_as_png(img2, png_filename_format, i);
  }
  return NULL;
}

int main(int argc, char *argv[])
{
  if (argc < 1)
    exit(1);
  if (atoi(argv[4]) != 0 && atoi(argv[4]) != 1) 
    exit(1);
  if (argc != 5)
  {
    fprintf(stderr, "usage: %s <nb-steps> <img-width> <img-height> <save-img>\n", argv[0]);
    exit(1);
  }

  int nb_steps = atoi(argv[1]);
  int width = atoi(argv[2]);
  int height = atoi(argv[3]);
  int save_img = atoi(argv[4]);

  img1 = alloc_img(width, height);
  img2 = alloc_img(width, height);


  srand(1);
  for (int i = 1; i < N_BODIES; ++i)
  {
    double dist = bodies[i].x;
    double angle = 2 * M_PI * (double)rand() / RAND_MAX;
    bodies[i].x = dist * cos(angle);
    bodies[i].y = dist * sin(angle);

    double rot_speed = bodies[i].vx;
    bodies[i].vx = -rot_speed * bodies[i].y;
    bodies[i].vy = rot_speed * bodies[i].x;
  }

  // clean files
  remove(stats_filename);
  char filename[256];
  if (save_img)
  {
    for (int i = 0; i < nb_steps; ++i)
    {
      snprintf(filename, 256, png_filename_format, i);
      remove(filename);
    }
  }

  sem_init(&sem_img_gen, 0, 0);
  sem_init(&sem_img_blur, 0, 0);
  sem_init(&sem_img_gray, 0, 0);
  sem_init(&sem_img_stats, 0, 0);
  if (save_img) sem_init(&sem_img_save, 0, 0);


  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1)
  {
    perror("clock_gettime");
    exit(1);
  }


  pthread_t threads[NUM_THREADS + save_img];
  pthread_create(&threads[0], NULL, f_simu, NULL);
  pthread_create(&threads[1], NULL, f_img_gen, NULL);
  pthread_create(&threads[2], NULL, f_img_blur, NULL);
  pthread_create(&threads[3], NULL, f_img_gray, NULL);
  pthread_create(&threads[4], NULL, f_img_stats, NULL);
  if (save_img) pthread_create(&threads[5], NULL, f_img_save, NULL);

  for (int i = 0; i < NUM_THREADS + save_img; i++)
  {
    pthread_join(threads[i], NULL);
  }

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1)
  {
    perror("clock_gettime");
    exit(1);
  }

  int64_t total_ns = ns_diff(&t0, &t1);
  print_elapsed_time_stats(total_ns);

  sem_destroy(&sem_img_gen);
  sem_destroy(&sem_img_blur);
  sem_destroy(&sem_img_gray);
  sem_destroy(&sem_img_stats);
  if (save_img)
    sem_destroy(&sem_img_save);

  free_img(img1);
  img1 = NULL;
  free_img(img2);
  img2 = NULL;

  return 0;
}
