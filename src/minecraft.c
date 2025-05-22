#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <ncurses.h>
#include <sys/time.h>
#include <stdlib.h>

const int WIDTH = 100;
const int HEIGHT = 30;
const int FPS = 10;
const float TPF = 1.0f/FPS;
const float Pi = 3.1415926f;
const int WORLD_SIZE_X = 10;
const int WORLD_SIZE_Y = 10;
const int WORLD_SIZE_Z = 2;
const float FOCAL_LENGTH = 0.5f;
const float INTRINSICS[9] = {45.0f, 0, WIDTH/2,
                            0, 15.0f, HEIGHT/2,
                            0, 0, 1.0f};
const float INIT_CAMERA_POS[5] = {-0.0f,0,20.0f,0,0};
const float MARGIN = 1e-8;

// block related
const float blockX = 10;
const float blockY = 10;
const float blockZ = 10;
const int strideX = 11;
const int strideY = 11;
const int strideZ = 11;
const int sampleX = 20;
const int sampleY = 20;
const int sampleZ = 20;

const char* SHADES = " .:-=+*#%@";
typedef struct {
    char ch;
    int color;
} Pixel;

typedef struct {
    int id_x;
    int id_y;
    int id_z;
    int surface_id;
} Block;

void dbg_print(char* s){
    printf("%s",s);
    fflush(stdout);
}

void display(Pixel* frame){
    mvprintw(2,0,"");
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        attron(COLOR_PAIR(frame[i].color));
        printw("%c", frame[i].ch);
        attroff(COLOR_PAIR(frame[i].color));
        if ((i + 1) % WIDTH == 0) printw("\n");
    }
    refresh();
}

float degree_to_rad(int degree){
    // constrain degree to be -90~90
    degree = degree < -90 ? -90 : degree;
    degree = degree > 90 ? 90 : degree;
    return (float)degree / 180 * Pi;
}

long long get_time_ms(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000LL + tv.tv_usec / 1000.0;
}

void matmul(const float* A, const float* B, float* C, int M, int K, int N){
    for(int i=0;i<M;i++){
        for(int k=0;k<K;k++){
            for(int j=0;j<N;j++){
                C[i*N + j] += A[i*K + k] * B[k*N + j];
            }
        }
    }
}

void transpose(float* A, float* B, int M, int N){
    // A M*N B N*M
    for(int i=0;i<M;i++){
        for(int j=0;j<N;j++){
            B[j*M + i] = A[i*N + j];
        }
    }
}

void construct_transform(float* R, float* T, float* M){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            M[i*4+j] = R[i*3+j];
        }
        M[i*4+3] = T[i];
    }
    M[15] = 1;
}

void get_R_t(float* pos, float* R, float* T){
    float x = pos[0];
    float y = pos[1];
    float z = pos[2];
    float yaw = pos[3];
    float pitch = pos[4];
    float tmp_R[3*3] = {cos(yaw) * cos(pitch), -sin(yaw), cos(yaw) * sin(pitch),
                    sin(yaw) * cos(pitch), cos(yaw), sin(yaw)*sin(pitch),
                    -sin(pitch), 0, cos(pitch),
                    };
    memcpy(R, tmp_R, sizeof(tmp_R));
    float tmp_T[3*1] = {x,y,z};
    memcpy(T, tmp_T, sizeof(tmp_T));
}

void xyz_to_uv(float* pos, float* obj, float* uv){
    float R_c_w[3*3] = {0};
    float T_c_w[3*1] = {0};
    get_R_t(pos, R_c_w, T_c_w);

    float R_w_c[3*3] = {0};
    transpose(R_c_w, R_w_c, 3, 3);
    float T_w_c[3*1] = {0};
    float neg_T_c_w[3*1] = {-T_c_w[0], -T_c_w[1], -T_c_w[2]};
    float M_w_c[4*4] = {0};
    matmul(R_w_c, neg_T_c_w, T_w_c, 3, 3, 1);
    
    construct_transform(R_w_c, T_w_c, M_w_c);

    float P_c[4*1] = {0};
    matmul(M_w_c, obj, P_c, 4, 4, 1);
    // this is the cam coord, we assume z axis perpendicular to image frame
    float z = P_c[0];
    float x = -P_c[2];
    float y = P_c[1];
    if( z <= 0+MARGIN ){
        uv[2] = -1;
    } else {
        uv[0] = (-INTRINSICS[0]*y/(z+FOCAL_LENGTH) + INTRINSICS[2]);
        uv[1] = (INTRINSICS[4]*x/(z+FOCAL_LENGTH) + INTRINSICS[5]);
        uv[2] = z;
    }
}

bool is_highlight_pixel(int x, int y){
    if (x > WIDTH/2 - 2 && x < WIDTH/2 + 2 && y > HEIGHT/2 - 2 && y < HEIGHT/2 + 2){
        return true;
    }
    return false;
}

void plot_block_to_frame(int id_x, 
                        int id_y, 
                        int id_z, 
                        Pixel* frame, 
                        float* pos, 
                        float* depth_map, 
                        Block* highlight_blk
                        ){
    // we only need 3 surface for one block
    // z == blockZ
    // todo: refactor to be simpler
    int block_x = id_x * strideX;
    int block_y = id_y * strideY;
    int block_z = id_z * strideZ;
    
    for(int p=0;p<sampleX;p++){
        float x_offset = (float)p * (float)blockX / (float)sampleX;
        for(int q=0;q<sampleY;q++){
            float y_offset = (float)q * (float)blockY / (float)sampleY;
            float obj[] = {(float)block_x + x_offset, (float)block_y + y_offset, (float)block_z + blockZ, 1.0f};
            float uv[3] = {0,0,0}; // [u ,v, depth]
            xyz_to_uv(pos, obj, uv);
            float depth = uv[2];
            int x = uv[0];
            int y = uv[1];
            if(depth != -1 && x > 0 && y > 0 && x < WIDTH && y < HEIGHT){
                // printf("pixel at %f, %f\n", uv[0], uv[1]);
                if(depth_map[y*WIDTH + x] <= 0 + MARGIN || depth < depth_map[y*WIDTH + x]){
                    frame[y*WIDTH + x].ch = '+';
                    depth_map[y*WIDTH +x] = depth;
                    // find a surface to highlight
                    if (is_highlight_pixel(x, y)){
                        highlight_blk->id_x = id_x;
                        highlight_blk->id_y = id_y;
                        highlight_blk->id_z = id_z;
                        highlight_blk->surface_id = 0;
                    }
                }
            }
        }
    }
    // x == 0
    for(int q=0;q<sampleY;q++){
        float y_offset = (float)q * (float)blockY / (float)sampleY;
        for (int w=0;w<sampleZ;w++){
            float z_offset = (float)w * (float)blockZ / (float)sampleZ;
            float obj[] = {(float)block_x, (float)block_y + y_offset, (float)block_z + z_offset, 1.0f};
            float uv[3] = {0,0,0}; // [u ,v, depth]
            xyz_to_uv(pos, obj, uv);
            float depth = uv[2];
            int x = uv[0];
            int y = uv[1];
            if(depth != -1 && x > 0 && y > 0 && x < WIDTH && y < HEIGHT){
                // printf("pixel at %f, %f\n", uv[0], uv[1]);
                if(depth_map[y*WIDTH + x] <= 0 + MARGIN || depth < depth_map[y*WIDTH + x]){
                    frame[y*WIDTH + x].ch = '*';
                    depth_map[y*WIDTH +x] = depth;
                    if (is_highlight_pixel(x, y)){
                        highlight_blk->id_x = id_x;
                        highlight_blk->id_y = id_y;
                        highlight_blk->id_z = id_z;
                        highlight_blk->surface_id = 1;
                    }
                }
            }
        }
    }
    // y == 0
    for(int p=0;p<sampleX;p++){
        float x_offset = (float)p * (float)blockX / (float)sampleX;
        for (int w=0;w<sampleZ;w++){
            float z_offset = (float)w * (float)blockZ / (float)sampleZ;
            float obj[] = {(float)block_x + x_offset, (float)block_y, (float)block_z + z_offset, 1.0f};
            float uv[3] = {0,0,0}; // [u ,v, depth]
            xyz_to_uv(pos, obj, uv);
            float depth = uv[2];
            int x = uv[0];
            int y = uv[1];
            if(depth != -1 && x > 0 && y > 0 && x < WIDTH && y < HEIGHT){
                // printf("pixel at %f, %f\n", uv[0], uv[1]);
                if(depth_map[y*WIDTH + x] <= 0 + MARGIN || depth < depth_map[y*WIDTH + x]){
                    frame[y*WIDTH + x].ch = '%';
                    depth_map[y*WIDTH +x] = depth;
                    if (is_highlight_pixel(x, y)){
                        highlight_blk->id_x = id_x;
                        highlight_blk->id_y = id_y;
                        highlight_blk->id_z = id_z;
                        highlight_blk->surface_id = 2;
                    }
                }
            }
        }
    }
}

void plot_highlight_block(
                        Pixel* frame, 
                        float* pos, 
                        Block* highklight_blk){
    int block_x = highklight_blk->id_x * strideX;
    int block_y = highklight_blk->id_y * strideY;
    int block_z = highklight_blk->id_z * strideZ;
    if(highklight_blk->surface_id==0){
        for(int p=0;p<sampleX;p++){
            float x_offset = (float)p * (float)blockX / (float)sampleX;
            for(int q=0;q<sampleY;q++){
                float y_offset = (float)q * (float)blockY / (float)sampleY;
                float obj[] = {(float)block_x + x_offset, (float)block_y + y_offset, (float)block_z + blockZ, 1.0f};
                float uv[3] = {0,0,0}; // [u ,v, depth]
                xyz_to_uv(pos, obj, uv);
                float depth = uv[2];
                int x = uv[0];
                int y = uv[1];
                if(depth != -1 && x > 0 && y > 0 && x < WIDTH && y < HEIGHT){
                    frame[y*WIDTH + x].color = 1;
                }
            }
        }
    } else if (highklight_blk->surface_id==1){
        for(int q=0;q<sampleY;q++){
            float y_offset = (float)q * (float)blockY / (float)sampleY;
            for (int w=0;w<sampleZ;w++){
                float z_offset = (float)w * (float)blockZ / (float)sampleZ;
                float obj[] = {(float)block_x, (float)block_y + y_offset, (float)block_z + z_offset, 1.0f};
                float uv[3] = {0,0,0}; // [u ,v, depth]
                xyz_to_uv(pos, obj, uv);
                float depth = uv[2];
                int x = uv[0];
                int y = uv[1];
                if(depth != -1 && x > 0 && y > 0 && x < WIDTH && y < HEIGHT){
                    // printf("pixel at %f, %f\n", uv[0], uv[1]);
                    frame[y*WIDTH + x].color = 1;
                }
            }
        }
    } else if (highklight_blk->surface_id==2){
        for(int p=0;p<sampleX;p++){
            float x_offset = (float)p * (float)blockX / (float)sampleX;
            for (int w=0;w<sampleZ;w++){
                float z_offset = (float)w * (float)blockZ / (float)sampleZ;
                float obj[] = {(float)block_x + x_offset, (float)block_y, (float)block_z + z_offset, 1.0f};
                float uv[3] = {0,0,0}; // [u ,v, depth]
                xyz_to_uv(pos, obj, uv);
                float depth = uv[2];
                int x = uv[0];
                int y = uv[1];
                if(depth != -1 && x > 0 && y > 0 && x < WIDTH && y < HEIGHT){
                    frame[y*WIDTH + x].color = 1;
                }
            }
        }
    }
}

void map_world_to_cam_view(Pixel* frame, float* pos, int* occupied_map, Block* highlight_blk){
    float depth_map[HEIGHT*WIDTH] = {0.0f};
    for(int i=0;i<WORLD_SIZE_X;i++){
        for(int j=0;j<WORLD_SIZE_Y;j++){
            // todo: project only 4 points and fill in the rest
            for(int k=0;k<WORLD_SIZE_Z;k++){
                if(occupied_map[k * WORLD_SIZE_X * WORLD_SIZE_Y + j * WORLD_SIZE_X + i] == 1){
                    plot_block_to_frame(i, j, k, frame, pos, depth_map, highlight_blk);
                }
            }
        }
    }
    // highlight the selected block
    plot_highlight_block(frame, pos, highlight_blk);
}

void update_world(int* occupied_map, int mouse_stat, Block* highlight_blk){
    int id_x = highlight_blk->id_x;
    int id_y = highlight_blk->id_y;
    int id_z = highlight_blk->id_z;
    if (mouse_stat==-1) return;
    if (mouse_stat==1) {
        occupied_map[id_z * WORLD_SIZE_X * WORLD_SIZE_Y + id_y * WORLD_SIZE_X + id_x] = 0;
        return;
    }
    
    if(highlight_blk->surface_id==0 && id_z+1<WORLD_SIZE_Z){
        occupied_map[(id_z+1) * WORLD_SIZE_X * WORLD_SIZE_Y + id_y * WORLD_SIZE_X + id_x] = 1;
    } else if (highlight_blk->surface_id==1 && id_x-1>=0){
        occupied_map[id_z * WORLD_SIZE_X * WORLD_SIZE_Y + id_y * WORLD_SIZE_X + id_x-1] = 1;
    } else if (highlight_blk->surface_id==2 && id_y-1>=0){
        occupied_map[id_z * WORLD_SIZE_X * WORLD_SIZE_Y + (id_y-1) * WORLD_SIZE_X + id_x] = 1;
    }
}

void generate_frame(Pixel* frame, float* camera_pos, int* occupied_map, int mouse_stat){
    // generate M * N world
    // transform a point from (i,j,0) to camera axis (x,y) given camera direction yaw: yawha, pitch: pitch, row: 0
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        frame[i].ch = '@';      // Blackest shade
        frame[i].color = 2;    // ANSI black foreground
    }
    Block highlight_blk = {0,0,0,-1};
    map_world_to_cam_view(frame, camera_pos, occupied_map, &highlight_blk);
    update_world(occupied_map, mouse_stat, &highlight_blk);
}

void update_camera_pos_t(float* relative_pos, float* camera_pos){
    float R_c_w[3*3] = {0};
    float T_c_w[3*1] = {0};
    float tmp_camera_pos[5*1] = {0};
    // banned pitch to make the transform limited to vertical
    memcpy(tmp_camera_pos, camera_pos, sizeof(tmp_camera_pos));
    tmp_camera_pos[4] = 0;
    get_R_t(tmp_camera_pos, R_c_w, T_c_w);
    float M_c_w[4*4] = {0};
    construct_transform(R_c_w, T_c_w, M_c_w);
    float P_w[4*1] = {0,0,0,1.0f};
    matmul(M_c_w, relative_pos, P_w, 4, 4, 1);
    camera_pos[0] = P_w[0];
    camera_pos[1] = P_w[1];
    camera_pos[2] = P_w[2];
}


int main(){
    MEVENT event;

    initscr();              // Start curses mode
    cbreak();               // Line buffering disabled
    noecho();               // Don't echo pressed keys
    curs_set(false);
    start_color();
    use_default_colors();
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_BLUE, COLOR_BLACK);
    keypad(stdscr, TRUE);   // Enable function keys and arrow keys
    mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);
    printf("\033[?1003h\n");  // Enable mouse move events (Xterm style)
    printw("Press 'q' to quit.\n");
    refresh();
    
    int time = 5; // in seconds
    long i = 0;
    long total_frame = time * FPS;
    long long prev_time_ms = get_time_ms();
    long long tpf_ms = (long long)(TPF * 1000.0f);
    Pixel* frame = malloc(HEIGHT*WIDTH * sizeof(Pixel));
    int occupied_map[WORLD_SIZE_X * WORLD_SIZE_Y * WORLD_SIZE_Z] = {0};
    memset(occupied_map, 0, sizeof(occupied_map));
    // for init state, we have a floor
    for (int y = 0; y < WORLD_SIZE_Y; y++) {
        for (int x = 0; x < WORLD_SIZE_X; x++) {
            int index = x + y * WORLD_SIZE_X + 0 * WORLD_SIZE_X * WORLD_SIZE_Y;
            occupied_map[index] = 1;
        }
    }
    
    float camera_pos[5];
    memcpy(camera_pos, INIT_CAMERA_POS, sizeof(camera_pos));
    
    // todo: use key_stat to move two directions same time e.g. a and w 
    // int key_stat[6] = {0}; // q:0 w:1 s:2 a:3 d:4 KEY_MOUSE:5
    int mouse_stat = -1; // -1 : none 0 : left button 1 :right button
    while(1){
        int ch = getch();
        
        if (ch == 'q')
            break;
        
        float relative_pos[4*1] = {0,0,0,1.0f};
        if (ch == 'w'){
            relative_pos[0] += 1;
        } else if (ch == 's'){
            relative_pos[0] += -1;
        }

        if (ch == 'a'){
            relative_pos[1] += 1;
        } else if (ch == 'd'){
            relative_pos[1] += -1;
        }

        update_camera_pos_t(relative_pos, camera_pos);

        if (ch == KEY_MOUSE) {
            if (getmouse(&event) == OK) {
                if (event.bstate & REPORT_MOUSE_POSITION) {
                    // width 
                    float x_axis_scale = 1.0f / (WIDTH/2) * 180.0f;
                    float y_axis_scale = 1.0f / (HEIGHT/2) * 90.0f;
                    camera_pos[3] = degree_to_rad(-(event.x - WIDTH/2) * x_axis_scale);
                    camera_pos[4] = degree_to_rad((event.y - HEIGHT/2) * y_axis_scale);
                    printw("%f, %f", camera_pos[3], camera_pos[4]);
                }
                if (event.bstate & BUTTON1_CLICKED)
                    mouse_stat = 0;
                else if (event.bstate & BUTTON3_CLICKED)
                    mouse_stat = 1;
            }
        }

        long long curr_time_ms = get_time_ms();
        if(curr_time_ms > prev_time_ms + tpf_ms){
            prev_time_ms = curr_time_ms;
            generate_frame(frame, camera_pos, occupied_map, mouse_stat);
            display(frame);
            mouse_stat = -1;
        }
    }
    printf("\033[?1003l\n");

    endwin();   // End curses mode
    return 0;
}