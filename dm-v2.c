#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/syscall.h>

#include "tasks.h"

#define NUM_THREADS 10

typedef struct {
  int step;
  void (*function)(int);
} Task;

typedef struct TaskNode {
  Task task;
  struct TaskNode *next;
} TaskNode;

// Déclaration globale et des semaphores
int nb_steps, width, height, save_img;
struct Image **tab_img1, **tab_img2;
struct ImageStats stats;
const char *stats_filename = "./img-stats.csv";
const char *png_filename_format = "./img%03d.png";
struct Body **tab_bodies;
struct Body body_base[N_BODIES] = {
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
TaskNode *tasks = NULL;
pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t task_cond = PTHREAD_COND_INITIALIZER;
int stop_threads = 0;

void addTask(void (*f)(int), int step)
{
  TaskNode *newNode = malloc(sizeof(TaskNode));
  newNode->task.function = f;
  newNode->task.step = step;
  newNode->next = NULL;

  pthread_mutex_lock(&task_mutex);
  if (tasks == NULL)
  {
    tasks = newNode;
  }
  else
  {
    TaskNode *current = tasks;
    while (current->next != NULL) current = current->next;
    current->next = newNode;
  }
  pthread_cond_signal(&task_cond);
  pthread_mutex_unlock(&task_mutex);
}

Task get_task()
{
  
  pthread_mutex_lock(&task_mutex);
  while(tasks == NULL && !stop_threads) pthread_cond_wait(&task_cond, &task_mutex);
  if(stop_threads && tasks == NULL)
  {
    pthread_mutex_unlock(&task_mutex);
    return (Task){NULL, -1};
  }

  TaskNode *task_node = tasks;
  tasks = tasks->next;
  Task task = task_node->task;
  free(task_node);
  pthread_mutex_unlock(&task_mutex);
  return task;
}

void f_simu(int step);

void f_img_save(int step)
{
  save_img_as_png(tab_img2[step], png_filename_format, step);
}

void f_img_stats(int step)
{
  compute_image_statistics(tab_img1[step], &stats);
  save_stats(&stats, stats_filename, step);
  if (step + 1 < nb_steps)  addTask(f_simu, step + 1);
  else
  {
    pthread_mutex_lock(&task_mutex);
    stop_threads = 1;
    pthread_cond_broadcast(&task_cond); // Réveille tous les threads bloqués
    pthread_mutex_unlock(&task_mutex);
  }
}

void f_img_gray(int step)
{ 
  convert_to_grayscale(tab_img2[step], tab_img1[step]);
  addTask(f_img_stats, step);
}

void f_img_blur(int step)
{
  apply_gaussian_blur(tab_img1[step], tab_img2[step]);
  addTask(f_img_gray, step);
  if (save_img) addTask(f_img_save, step);
}

void f_img_gen(int step)
{
  generate_image_from_bodies(tab_bodies[step], N_BODIES, tab_img1[step]);
  addTask(f_img_blur, step);
}

void f_simu(int step)
{
  //pid_t tid = syscall(SYS_gettid);
  //printf("%d | f_simu [%d]\n", tid, step);
  if (step>0) tab_bodies[step] = tab_bodies[step-1];
  simulate_n_bodies(tab_bodies[step], N_BODIES, 1.0);
  addTask(f_img_gen, step);
}


void *task(void *arg)
{
  while (1)
  {
    Task task = get_task();
    if (stop_threads) break;
    task.function(task.step);
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

  nb_steps = atoi(argv[1]);
  width = atoi(argv[2]);
  height = atoi(argv[3]);
  save_img = atoi(argv[4]);

  tab_img1 = malloc(nb_steps * sizeof(struct Image *));
  tab_img2 = malloc(nb_steps * sizeof(struct Image *));
  for(int i = 0; i < nb_steps; i++)
  {
    tab_img1[i] = alloc_img(width, height);
    tab_img2[i] = alloc_img(width, height);
  }

  tab_bodies = malloc(nb_steps * sizeof(struct Body[N_BODIES]));
  srand(1);
  for (int i = 1; i < N_BODIES; ++i)
  {
    double dist = body_base[i].x;
    double angle = 2 * M_PI * (double)rand() / RAND_MAX;
    body_base[i].x = dist * cos(angle);
    body_base[i].y = dist * sin(angle);

    double rot_speed = body_base[i].vx;
    body_base[i].vx = -rot_speed * body_base[i].y;
    body_base[i].vy = rot_speed * body_base[i].x;
  }  
  tab_bodies[0] = body_base;

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


  struct timespec t0, t1;
  if (clock_gettime(CLOCK_BOOTTIME, &t0) == -1)
  {
    perror("clock_gettime");
    exit(1);
  }

  addTask(f_simu, 0);
  pthread_t threads[NUM_THREADS];
  for(int i = 0; i < NUM_THREADS; ++i){
    pthread_create(&threads[i], NULL, task, NULL);
    //printf("Thread %d created\n", i);
  }


  for(int i = 0; i < NUM_THREADS; ++i){
    pthread_join(threads[i], NULL);
  }

  if (clock_gettime(CLOCK_BOOTTIME, &t1) == -1)
  {
    perror("clock_gettime");
    exit(1);
  }

  int64_t total_ns = ns_diff(&t0, &t1);
  print_elapsed_time_stats(total_ns);

  free(tab_bodies);
  for (int i = 0; i < nb_steps; i++)
  {
    free_img(tab_img1[i]);
    free_img(tab_img2[i]);
  }
  free(tab_img1);
  free(tab_img2);

  return 0;
}