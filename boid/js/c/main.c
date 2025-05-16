#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <stdlib.h>
#include <math.h>

static SDL_Window* window = NULL;
static SDL_Renderer* renderer = NULL;
static Uint64 last_time = 0;

#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

#define START_SPEED_MIN 60 
#define START_SPEED_MAX 120 

#define DETECTION_RADIOUS 40 
#define ALIGN_STRENGHT 0.01 

#define SEPARATION_RADIOUS 10 
#define SEPARATION_STRENGTH 0.3 

#define COHESION_STRENGTH 0.02

#define FEAR_STRENGTH 1


#define EAGLE_HUNT_STRENGTH 0.03
#define EAGLE_VISION_RADIOUS 150

#define NUM_BOIDS 20000
#define NUM_EAGLES 10

#define CELL_SIZE (DETECTION_RADIOUS + 10)
#define GRID_COLS (WINDOW_WIDTH / CELL_SIZE + 1)
#define GRID_ROWS (WINDOW_HEIGHT / CELL_SIZE + 1)

#define MAX_BOIDS_PER_CELL 20  

struct Cell {
    int boids[MAX_BOIDS_PER_CELL];
    int count;
};
struct Cell grid[GRID_COLS][GRID_ROWS];

struct Boid {
    float x;
    float y;
    float vx;
    float vy;
};

struct Eagle {
    float x;
    float y;
    float vx;
    float vy;
};

struct Boid boids[NUM_BOIDS];
struct Eagle eagles[NUM_EAGLES];


void update_grid() {

    // Clear the grid
    for (int x = 0; x < GRID_COLS; x++) {
        for (int y = 0; y < GRID_ROWS; y++) {
            grid[x][y].count = 0;
        }
    }

    // Assign boids to grid cells
    for (int i = 0; i < NUM_BOIDS; i++) {
        int cell_x = (int)(boids[i].x / CELL_SIZE);
        int cell_y = (int)(boids[i].y / CELL_SIZE);

        if (cell_x >= 0 && cell_x < GRID_COLS && cell_y >= 0 && cell_y < GRID_ROWS) {
            struct Cell* cell = &grid[cell_x][cell_y];
            if (cell->count < MAX_BOIDS_PER_CELL) {
                cell->boids[cell->count++] = i;
            }
        }
    }
}

void update_boid(int boidIndex) {
    int count_neighbors = 0, count_sep = 0;
    float sum_vx = 0, sum_vy = 0;
    float sum_x = 0, sum_y = 0;
    float sum_sep_x = 0, sum_sep_y = 0;

    // Get boid's cell
    int cell_x = (int)(boids[boidIndex].x / CELL_SIZE);
    int cell_y = (int)(boids[boidIndex].y / CELL_SIZE);

    // Check neighboring cells
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            int nx = cell_x + dx;
            int ny = cell_y + dy;
            if (nx >= 0 && nx < GRID_COLS && ny >= 0 && ny < GRID_ROWS) {
                struct Cell* cell = &grid[nx][ny];
                for (int k = 0; k < cell->count; k++) {
                    int j = cell->boids[k];
                    if (j == boidIndex) continue;

                    float dx = boids[j].x - boids[boidIndex].x;
                    float dy = boids[j].y - boids[boidIndex].y;
                    float d2 = dx * dx + dy * dy;

                    if (d2 < DETECTION_RADIOUS * DETECTION_RADIOUS) {
                        count_neighbors++;
                        sum_vx += boids[j].vx;
                        sum_vy += boids[j].vy;
                        sum_x += boids[j].x;
                        sum_y += boids[j].y;
                    }
                    if (d2 < SEPARATION_RADIOUS * SEPARATION_RADIOUS) {
                        count_sep++;
                        sum_sep_x += boids[j].x;
                        sum_sep_y += boids[j].y;
                    }
                }
            }
        }
    }

    // Alignment
    if (count_neighbors > 0) {
        float avg_vx = sum_vx / count_neighbors;
        float avg_vy = sum_vy / count_neighbors;
        boids[boidIndex].vx += (avg_vx - boids[boidIndex].vx) * ALIGN_STRENGHT;
        boids[boidIndex].vy += (avg_vy - boids[boidIndex].vy) * ALIGN_STRENGHT;
    }

    // Cohesion
    if (count_neighbors > 0) {
        float avg_x = sum_x / count_neighbors;
        float avg_y = sum_y / count_neighbors;
        float cohesion_x = avg_x - boids[boidIndex].x;
        float cohesion_y = avg_y - boids[boidIndex].y;
        boids[boidIndex].vx += cohesion_x * COHESION_STRENGTH;
        boids[boidIndex].vy += cohesion_y * COHESION_STRENGTH;
    }

    // Separation
    if (count_sep > 0) {
        float avg_sep_x = sum_sep_x / count_sep;
        float avg_sep_y = sum_sep_y / count_sep;
        float separation_x = boids[boidIndex].x - avg_sep_x;
        float separation_y = boids[boidIndex].y - avg_sep_y;
        boids[boidIndex].vx += separation_x * SEPARATION_STRENGTH;
        boids[boidIndex].vy += separation_y * SEPARATION_STRENGTH;
    }

    // Limit Speed
    float speed = sqrtf(boids[boidIndex].vx * boids[boidIndex].vx + boids[boidIndex].vy * boids[boidIndex].vy);
    if (speed > START_SPEED_MAX) {
        boids[boidIndex].vx = (boids[boidIndex].vx / speed) * START_SPEED_MAX;
        boids[boidIndex].vy = (boids[boidIndex].vy / speed) * START_SPEED_MAX;
    }
}

void fear(int boidIndex)
{
    int count = 0;
    float xx = 0, yy = 0;
    for (int i = 0; i < NUM_EAGLES; i++) {
        float dx = boids[boidIndex].x - eagles[i].x;
        float dy = boids[boidIndex].y - eagles[i].y;
        float d2 = dx * dx + dy * dy;
        if (d2 < DETECTION_RADIOUS * DETECTION_RADIOUS) {
            xx += eagles[i].x;
            yy += eagles[i].y;
            count++;
        }
    }
    if (count > 0) {
        float sx = boids[boidIndex].x - (xx / count);
        float sy = boids[boidIndex].y - (yy / count);
        boids[boidIndex].vx += sx * FEAR_STRENGTH;
        boids[boidIndex].vy += sy * FEAR_STRENGTH;
    }
}

void hunt(int eagleIndex)
{
    int count = 0;
    float xx = 0, yy = 0;
    for (int i = 0; i < NUM_BOIDS; i++) {
        float dx = boids[i].x - eagles[eagleIndex].x;
        float dy = boids[i].y - eagles[eagleIndex].y;
        float d2 = dx * dx + dy * dy;
        if (d2 < EAGLE_VISION_RADIOUS * EAGLE_VISION_RADIOUS) {
            xx += boids[i].x;
            yy += boids[i].y;
            count++;
        }
    }
    if (count > 0) {
        float cx = (xx / count) - eagles[eagleIndex].x;
        float cy = (yy / count) - eagles[eagleIndex].y;
        eagles[eagleIndex].vx += cx * EAGLE_HUNT_STRENGTH;
        eagles[eagleIndex].vy += cy * EAGLE_HUNT_STRENGTH;
    }
    float speed = sqrtf(eagles[eagleIndex].vx * eagles[eagleIndex].vx + eagles[eagleIndex].vy * eagles[eagleIndex].vy);
    if (speed > START_SPEED_MAX) {
        eagles[eagleIndex].vx = (eagles[eagleIndex].vx / speed) * START_SPEED_MAX;
        eagles[eagleIndex].vy = (eagles[eagleIndex].vy / speed) * START_SPEED_MAX;
    }
    if (speed > 0) {
        float desired_speed = (START_SPEED_MIN + START_SPEED_MAX) / 2;
        eagles[eagleIndex].vx = (eagles[eagleIndex].vx / speed) * desired_speed;
        eagles[eagleIndex].vy = (eagles[eagleIndex].vy / speed) * desired_speed;
    }
}

void SDL_RenderFillCircle(SDL_Renderer* renderer, int x, int y, int radius)
{
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius))
                SDL_RenderPoint(renderer, x + dx, y + dy);
        }
    }
}

/* SDL Initialization */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[])
{
    SDL_SetAppMetadata("Boid Simulation", "1.0", "com.example.boids");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("Boid Simulation", WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer)) {
        SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    // Initialize boids
    for (int i = 0; i < NUM_BOIDS; i++) {
        boids[i].x = SDL_randf() * WINDOW_WIDTH;
        boids[i].y = SDL_randf() * WINDOW_HEIGHT;
        boids[i].vx = (rand() % (START_SPEED_MAX - START_SPEED_MIN + 1) + START_SPEED_MIN) * (SDL_randf() > 0.5f ? 1 : -1);
        boids[i].vy = (rand() % (START_SPEED_MAX - START_SPEED_MIN + 1) + START_SPEED_MIN) * (SDL_randf() > 0.5f ? 1 : -1);
    }
    // Initialize eagles
    for (int i = 0; i < NUM_EAGLES; i++) {
        eagles[i].x = SDL_randf() * WINDOW_WIDTH;
        eagles[i].y = SDL_randf() * WINDOW_HEIGHT;
        eagles[i].vx = (rand() % (START_SPEED_MAX - START_SPEED_MIN + 1) + START_SPEED_MIN) * (SDL_randf() > 0.5f ? 1 : -1);
        eagles[i].vy = (rand() % (START_SPEED_MAX - START_SPEED_MIN + 1) + START_SPEED_MIN) * (SDL_randf() > 0.5f ? 1 : -1);
    }
    last_time = SDL_GetTicks();
    return SDL_APP_CONTINUE;
}

/* Handle Events */
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event)
{
    if (event->type == SDL_EVENT_QUIT)
        return SDL_APP_SUCCESS;
    return SDL_APP_CONTINUE;
}

/* Main Render Loop */
SDL_AppResult SDL_AppIterate(void* appstate) {
    Uint64 now = SDL_GetTicks();
    float elapsed = ((float)(now - last_time)) / 1000.0f;
    last_time = now;

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    update_grid();  // Update spatial partitioning

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

    // Update boids using the grid
    for (int i = 0; i < NUM_BOIDS; i++) {
        update_boid(i);
        fear(i);
        boids[i].x += boids[i].vx * elapsed;
        boids[i].y += boids[i].vy * elapsed;

        // Wrap around edges
        if (boids[i].x > WINDOW_WIDTH) boids[i].x = 0;
        if (boids[i].y > WINDOW_HEIGHT) boids[i].y = 0;
        if (boids[i].x < 0) boids[i].x = WINDOW_WIDTH;
        if (boids[i].y < 0) boids[i].y = WINDOW_HEIGHT;

        SDL_RenderPoint(renderer, boids[i].x, boids[i].y);
    }

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    for (size_t i = 0; i < NUM_EAGLES; i++)
    {
        hunt(i);
        eagles[i].x += eagles[i].vx * elapsed;
        eagles[i].y += eagles[i].vy * elapsed;

        if (eagles[i].x > WINDOW_WIDTH) boids[i].x = 0;
        if (eagles[i].y > WINDOW_HEIGHT) boids[i].y = 0;
        if (eagles[i].x < 0) eagles[i].x = WINDOW_WIDTH;
        if (eagles[i].y < 0) eagles[i].y = WINDOW_HEIGHT;

        SDL_RenderFillCircle(renderer, eagles[i].x, eagles[i].y, 2);
    }

    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}


/* Cleanup */
void SDL_AppQuit(void* appstate, SDL_AppResult result)
{
    /* SDL cleans up the window/renderer automatically */
}
