#include "../header/functions.h"

char mode_start;

Camera *camera(0);
extern Arene *monArene(0);
extern int arene_validee=0;
extern int mode_camera=0; 

void write_in_queue(RT_QUEUE *, MessageToMon);

void f_server(void *arg) {
    int err;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    err = run_nodejs("/usr/local/bin/node", "/home/pi/Interface_Robot/server.js");

    if (err < 0) {
        printf("Failed to start nodejs: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } else {
#ifdef _WITH_TRACE_
        printf("%s: nodejs started\n", info.name);
#endif
        open_server();
        rt_sem_broadcast(&sem_serverOk);
    }
}

void f_sendToMon(void * arg) {
    int err;
    MessageToMon msg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {

#ifdef _WITH_TRACE_
        printf("%s : waiting for a message in queue\n", info.name);
#endif
        if (rt_queue_read(&q_messageToMon, &msg, sizeof (MessageToRobot), TM_INFINITE) >= 0) {
#ifdef _WITH_TRACE_
            printf("%s : message {%s,%s} in queue\n", info.name, msg.header, msg.data);
#endif

            send_message_to_monitor(msg.header, msg.data);
            free_msgToMon_data(&msg);
            rt_queue_free(&q_messageToMon, &msg);
        } else {
            printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void f_receiveFromMon(void *arg) {
    MessageFromMon msg;
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    do {
#ifdef _WITH_TRACE_
        printf("%s : waiting for a message from monitor\n", info.name);
#endif
        err = receive_message_from_monitor(msg.header, msg.data);
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
            if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_openComRobot);
            }
        } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
            if (msg.data[0] == DMB_START_WITHOUT_WD) { // Start robot
#ifdef _WITH_TRACE_
                printf("%s: message start robot\n", info.name);
#endif 
                rt_sem_v(&sem_startRobot);

            } else if ((msg.data[0] == DMB_GO_BACK)
                    || (msg.data[0] == DMB_GO_FORWARD)
                    || (msg.data[0] == DMB_GO_LEFT)
                    || (msg.data[0] == DMB_GO_RIGHT)
                    || (msg.data[0] == DMB_STOP_MOVE)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msg.data[0];
                rt_mutex_release(&mutex_move);
               
#ifdef _WITH_TRACE_
                printf("%s: message update movement with %c\n", info.name, move);
#endif

            }
        }
        else if(strcmp(msg.header, HEADER_MTS_CAMERA) == 0)
        {
            if (msg.data[0] == CAM_OPEN) {
                rt_sem_v(&sem_startCam);
                printf("Message d'ouverture recu\n");
            }
        }
    } while (err > 0);

}

void f_openComRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
        err = open_communication_robot();
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_startRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRobot\n", info.name);
#endif
        rt_sem_p(&sem_startRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startRobot arrived => Start robot\n", info.name);
#endif
        err = send_command_to_robot(DMB_START_WITHOUT_WD);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_move(void *arg) {
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
/*#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif*/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
/*#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif*/
        rt_task_wait_period(NULL);
/*#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
        printf("%s: move equals %c\n", info.name, move);
#endif*/
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            send_command_to_robot(move);
            rt_mutex_release(&mutex_move);
/*#ifdef _WITH_TRACE_
            printf("%s: the movement %c was sent\n", info.name, move);
#endif            */
        }
        rt_mutex_release(&mutex_robotStarted);
    }
}

void write_in_queue(RT_QUEUE *queue, MessageToMon msg) {
    void *buff;
    buff = rt_queue_alloc(&q_messageToMon, sizeof (MessageToMon));
    memcpy(buff, &msg, sizeof (MessageToMon));
    rt_queue_send(&q_messageToMon, buff, sizeof (MessageToMon), Q_NORMAL);
}



void f_openCamera (void *arg) {
    /* INIT */
    rt_sem_p(&sem_barrier, TM_INFINITE);
    printf("Attente sem_barriere\n");
    camera = (Camera *)malloc(sizeof(Camera));
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    printf("Attente sem_startCam\n");
    rt_sem_p(&sem_startCam, TM_INFINITE);
    printf("Sem_startCam recu\n");
    /*if (open_camera(camera)==-1){ //echec ouverture camera
        printf("Probleme camera \n");
        MessageToMon msg;
        set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
        write_in_queue(&q_messageToMon, msg);
    }
    else { //ouverture camera OK
        printf("Camera ouverte\n");
        mode_camera=1 ;
        
    }*/
    open_camera(camera);
    mode_camera=1;
}


void f_camera (void *arg){
    /* INIT */
    RT_TASK_INFO info;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    //rt_sem_p(&sem_cameraStarted, TM_INFINITE);
    /* PERIODIC START */
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    Image *image_in;
    //image_in = malloc(sizeof(Image));
    Image *image_out;
    MessageToMon msg;
    Position *maPosition ;
    printf("on entre dans le while pour l'acquisition des images\n");
    while (1) {
        rt_task_wait_period(NULL);
        if (mode_camera==1) { //en mode 1 la camera envoie les images (avec arene dessinee dessus si l'arene a ete validee)
            /*if (get_image(camera,image_in,NULL)==-1) {
                printf("Erreur capture image\n");
            }
            else {
             */
            printf("on est avant le get_image\n");
            get_image(camera,image_in,"/home/bertrana/Documents/4eme_annee/temps_reel/TP_temps_reel/raspberry.jpg");
            printf("on est après le get_image\n");
            /*if (arene_validee==1) {
                draw_arena(image_in, image_out, monArene);
            }
            else {*/
                image_out=image_in;
            //}
            set_msgToMon_header(&msg, HEADER_STM_IMAGE);
            set_msgToMon_data(&msg, image_out);
            send_message_to_monitor(msg.header,msg.data);
        }
        else if (mode_camera==2) { //Calcul de la position et envoi de la position et image avec position dessinee
            Image *imagePos;
            MessageToMon pos;
            get_image(camera,image_in,NULL);
            if (arene_validee==1) {
                if (detect_position(image_in, maPosition, monArene)==0) {
                    printf("Pas de robot detecte\n");
                }
                draw_arena(image_in, image_out, monArene);
            }
            else {
                if (detect_position(image_in, maPosition)==0) {
                    printf("Pas de robot detecte\n");
                }
                image_out=image_in;
            }
            draw_position(image_out, imagePos, maPosition);
            set_msgToMon_header(&msg, HEADER_STM_IMAGE);
            set_msgToMon_data(&msg, imagePos);
            send_message_to_monitor(msg.header,msg.data);
            set_msgToMon_header(&pos, HEADER_STM_POS);
            set_msgToMon_data(&pos, maPosition);
            send_message_to_monitor(pos.header,pos.data);
        }
        else if (mode_camera==3){
            Arene *rectangle;
            MessageToMon demande_arene;
            char *message;
            get_image(camera,image_in,NULL);
            detect_arena(image_in, rectangle);
            set_msgToMon_header(&msg, HEADER_STM_IMAGE);
            set_msgToMon_data(&msg, image_in);
            send_message_to_monitor(msg.header,msg.data);
            char* texte = "Validation de l'arene ? (y/n)\n";
            set_msgToMon_header(&demande_arene, HEADER_STM_MES);
            set_msgToMon_data(&demande_arene, (void*) texte);
            send_message_to_monitor(demande_arene.header,demande_arene.data);
            receive_message_from_monitor("MSG", message);
            if (message[0]='y'){
                arene_validee=1;
            }
            else {
                arene_validee=0;
            }
        }
    }
}
