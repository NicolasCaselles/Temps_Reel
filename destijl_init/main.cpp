/* 
 * File:   main.c
 * Author: pehladik
 *
 * Created on 23 décembre 2017, 19:45
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "./header/functions.h"

// Déclaration des taches
RT_TASK th_server;
RT_TASK th_sendToMon;
RT_TASK th_receiveFromMon;
RT_TASK th_openComRobot;
RT_TASK th_startRobot;
RT_TASK th_move;
RT_TASK th_batterie;
RT_TASK th_WatchComServer;
RT_TASK th_WatchComRobot;

// Déclaration des priorités des taches
int PRIORITY_TSERVER = 30;
int PRIORITY_TOPENCOMROBOT = 26;
int PRIORITY_TMOVE = 10;
int PRIORITY_TSENDTOMON = 25;
int PRIORITY_TRECEIVEFROMMON = 22;
int PRIORITY_TSTARTROBOT = 20;
int PRIORITY_TBATTERIE = 12;
int PRIORITY_TWATCHCOMSERVER = 9;
int PRIORITY_TWATCHCOMROBOT =29;

RT_MUTEX mutex_robotStarted;
RT_MUTEX mutex_move;
RT_MUTEX mutex_camera;
RT_MUTEX mutex_compteur;

// Déclaration des sémaphores
RT_SEM sem_barrier;
RT_SEM sem_openComRobot;
RT_SEM sem_serverOk;
RT_SEM sem_startRobot;
RT_SEM sem_errS;
RT_SEM sem_errR;
RT_SEM sem_probComm;

// Déclaration des files de message
RT_QUEUE q_messageToMon;

int MSG_QUEUE_SIZE = 10;

// Déclaration des ressources partagées
int etatCommMoniteur = 1;
int robotStarted = 0;
int mode_camera = 0;
char move = DMB_STOP_MOVE;

/**
 * \fn void initStruct(void)
 * \brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void initStruct(void);

/**
 * \fn void startTasks(void)
 * \brief Démarrage des tâches
 */
void startTasks(void);

/**
 * \fn void deleteTasks(void)
 * \brief Arrêt des tâches
 */
void deleteTasks(void);

int main(int argc, char **argv) {
    int err;
    //Lock the memory to avoid memory swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("#################################\n");
    printf("#      DE STIJL PROJECT         #\n");
    printf("#################################\n");

    initStruct();
    startTasks();
    rt_sem_broadcast(&sem_barrier);
    pause();
    deleteTasks();

    return 0;
}

void initStruct(void) {
    int err;
    /* Creation des mutex */
    if (err = rt_mutex_create(&mutex_compteur, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    /* Creation du semaphore */
    if (err = rt_sem_create(&sem_probComm, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_errS, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_errR, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    /* Creation des taches */
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
        if (err = rt_task_create(&th_batterie, "th_batterie", 0, PRIORITY_TBATTERIE, 0)) {
        printf("\n\n\n\nError task create: %s\n\n\n\n\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_WatchComServer, "th_WatchComServer", 0, PRIORITY_TWATCHCOMSERVER, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_WatchComRobot, "th_WatchComRobot", 0, PRIORITY_TWATCHCOMROBOT, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    /* Creation des files de messages */
    if (err = rt_queue_create(&q_messageToMon, "toto", MSG_QUEUE_SIZE * sizeof (MessageToRobot), MSG_QUEUE_SIZE, Q_FIFO)) {
        printf("Error msg queue create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
}

void startTasks() {

    int err;
    
    if (err = rt_task_start(&th_startRobot, &f_startRobot, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_receiveFromMon, &f_receiveFromMon, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, &f_sendToMon, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, &f_openComRobot, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, &f_move, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_batterie, &f_batterie, NULL)) {
        printf("\n\n\n\nError task start: %s\n\n\n\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_WatchComServer, &f_watchComServer, NULL)) {
        printf("\n\n\n\nError task start: %s\n\n\n\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_WatchComRobot, &f_watchComRobot, NULL)) {
        printf("\n\n\n\nError task start: %s\n\n\n\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_server, &f_server, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }                  
    

}

void deleteTasks() {
    rt_task_delete(&th_server);
    rt_task_delete(&th_openComRobot);
    rt_task_delete(&th_move);
    rt_task_delete(&th_batterie);
    rt_task_delete(&th_WatchComRobot);
    rt_task_delete(&th_WatchComServer);
}
