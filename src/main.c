#define ENET_IMPLEMENTATION
#include "enet.h"
#include "raylib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// ==================== КОНСТАНТЫ КАРТЫ ====================
#define MAP_WIDTH 12
#define MAP_HEIGHT 12
#define TILE_HEIGHT 0.2f
#define HEIGHT_STEP 1.0f
#define MAX_HEIGHT_DIFF 1  // Максимальная разница высот для проходимости

// ==================== КОНСТАНТЫ КАМЕРЫ ====================
#define CAMERA_RADIUS 14.0f
#define CAMERA_HEIGHT 12.0f
#define CAMERA_INITIAL_ANGLE (PI / 4.0f)  // 45 градусов - стандартная изометрия
#define CAMERA_ROTATION_STEP (PI / 4.0f)  // Поворот на 45 градусов
#define CAMERA_FOVY 8.0f

// ==================== КОНСТАНТЫ ИГРОКА ====================
#define PLAYER_START_X 6.0f
#define PLAYER_START_Z 6.0f
#define PLAYER_MOVE_SPEED 8.0f
#define PLAYER_SPRITE_WIDTH 48
#define PLAYER_SPRITE_HEIGHT 64

// ==================== НАПРАВЛЕНИЯ ДВИЖЕНИЯ ====================
typedef enum {
  DIR_UP = 0,
  DIR_UP_RIGHT,
  DIR_RIGHT,
  DIR_DOWN_RIGHT,
  DIR_DOWN,
  DIR_DOWN_LEFT,
  DIR_LEFT,
  DIR_UP_LEFT,
  DIR_COUNT  // = 8, используется для модульной арифметики
} Direction;

typedef struct {
  int dx;
  int dz;
} Dir2D;

// Базовые направления для стандартной изометрии (камера на 45°)
// При угле 45° камера находится в +X,+Z и смотрит на игрока
Dir2D baseDirections[DIR_COUNT] = {
  {-1, -1},  // DIR_UP: "вверх по экрану" = в дальний угол
  { 0, -1},  // DIR_UP_RIGHT: вправо-вверх
  { 1, -1},  // DIR_RIGHT: "вправо по экрану"
  { 1,  0},  // DIR_DOWN_RIGHT: вправо-вниз
  { 1,  1},  // DIR_DOWN: "вниз по экрану" = в ближний угол
  { 0,  1},  // DIR_DOWN_LEFT: влево-вниз
  {-1,  1},  // DIR_LEFT: "влево по экрану"
  {-1,  0}   // DIR_UP_LEFT: влево-вверх
};

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================
// Безопасный оператор остатка для отрицательных чисел
int safeMod(int a, int b) {
  return ((a % b) + b) % b;
}

int main(void)
{
  if (enet_initialize() != 0) {
    printf("An error occurred while initializing ENet.\n");
    return 1;
  }

  InitWindow(1600, 900, "Isometric Pixel Dungeon - Relative Movement");
  SetTargetFPS(60);

  float cameraAngle = CAMERA_INITIAL_ANGLE; 

  Camera3D camera = {0};
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  camera.fovy = CAMERA_FOVY; 
  camera.projection = CAMERA_ORTHOGRAPHIC; 

  // Генерация тестовой карты высот
  int heightMap[MAP_WIDTH][MAP_HEIGHT] = {0};
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      int distX = abs(x - 6);
      int distZ = abs(z - 6);
      int h = 4 - (distX + distZ) / 2;
      if (h < 0) h = 0; 
      if (h > 4) h = 4; 

      if (x == 3 && z == 3) h = 1; 
      if (x == 8 && z == 8) h = 4; 

      heightMap[x][z] = h;
    }
  }

  float targetX = PLAYER_START_X; 
  float targetZ = PLAYER_START_Z;
  float playerX = PLAYER_START_X;
  float playerZ = PLAYER_START_Z;

  while (!WindowShouldClose())
  {
    float dt = GetFrameTime();

    // Дискретный поворот камеры
    if (IsKeyPressed(KEY_Q)) {
      cameraAngle -= CAMERA_ROTATION_STEP;
    }
    if (IsKeyPressed(KEY_E)) {
      cameraAngle += CAMERA_ROTATION_STEP;
    }

    // Обработка движения
    Direction dirIndex = -1;

    // Диагонали (проверяем первыми)
    if (IsKeyPressed(KEY_W) && IsKeyPressed(KEY_A))      dirIndex = DIR_UP_LEFT;
    else if (IsKeyPressed(KEY_W) && IsKeyPressed(KEY_D)) dirIndex = DIR_UP_RIGHT;
    else if (IsKeyPressed(KEY_S) && IsKeyPressed(KEY_A)) dirIndex = DIR_DOWN_LEFT;
    else if (IsKeyPressed(KEY_S) && IsKeyPressed(KEY_D)) dirIndex = DIR_DOWN_RIGHT;
    // Основные направления
    else if (IsKeyPressed(KEY_W)) dirIndex = DIR_UP;
    else if (IsKeyPressed(KEY_D)) dirIndex = DIR_RIGHT;
    else if (IsKeyPressed(KEY_S)) dirIndex = DIR_DOWN;
    else if (IsKeyPressed(KEY_A)) dirIndex = DIR_LEFT;

    if (dirIndex != -1) {
      // Вычисляем, насколько повернута камера относительно базового угла
      int rotationSteps = (int)roundf((cameraAngle - CAMERA_INITIAL_ANGLE) / CAMERA_ROTATION_STEP);
      rotationSteps = safeMod(rotationSteps, DIR_COUNT);

      // Применяем поворот к направлению
      Direction worldDir = safeMod(dirIndex - rotationSteps, DIR_COUNT);

      targetX += baseDirections[worldDir].dx;
      targetZ += baseDirections[worldDir].dz;
    }

    // Проверка коллизий
    int intTargetX = (int)roundf(targetX);
    int intTargetZ = (int)roundf(targetZ);
    int currentPlayerX = (int)roundf(playerX);
    int currentPlayerZ = (int)roundf(playerZ);

    if (intTargetX < 0 || intTargetX >= MAP_WIDTH || 
        intTargetZ < 0 || intTargetZ >= MAP_HEIGHT) {
      targetX = playerX; 
      targetZ = playerZ;
    } 
    else if (heightMap[intTargetX][intTargetZ] == 0) {
      targetX = playerX; 
      targetZ = playerZ;
    }
    else if (abs(heightMap[intTargetX][intTargetZ] - heightMap[currentPlayerX][currentPlayerZ]) > MAX_HEIGHT_DIFF) {
      targetX = playerX; 
      targetZ = playerZ;
    }

    // Плавная интерполяция
    float step = PLAYER_MOVE_SPEED * dt;
    float dx = targetX - playerX;
    float dz = targetZ - playerZ;

    if (fabsf(dx) > 0.01f) playerX += (dx > 0 ? 1.0f : -1.0f) * fminf(step, fabsf(dx));
    else playerX = targetX;

    if (fabsf(dz) > 0.01f) playerZ += (dz > 0 ? 1.0f : -1.0f) * fminf(step, fabsf(dz));
    else playerZ = targetZ;

    // Обновление камеры
    int gridX = (int)roundf(playerX);
    int gridZ = (int)roundf(playerZ);
    float groundY = (float)heightMap[gridX][gridZ] * HEIGHT_STEP;

    camera.target = (Vector3){ playerX, groundY + 0.5f, playerZ };
    camera.position = (Vector3){
      camera.target.x + CAMERA_RADIUS * cosf(cameraAngle),
      camera.target.y + CAMERA_HEIGHT,
      camera.target.z + CAMERA_RADIUS * sinf(cameraAngle)
    };

    // DRAW
    BeginDrawing();
    ClearBackground((Color){30, 30, 40, 255}); 

    BeginMode3D(camera);

    for (int x = 0; x < MAP_WIDTH; x++) {
      for (int z = 0; z < MAP_HEIGHT; z++) {
        int h = heightMap[x][z];
        if (h > 0) {
          Vector3 pos = { (float)x, (float)h * HEIGHT_STEP, (float)z };
          unsigned char brightness = 80 + h * 30;
          Color tileColor = { brightness, brightness, brightness + 20, 255 };

          DrawCubeV(pos, (Vector3){1.0f, TILE_HEIGHT, 1.0f}, tileColor);
          DrawCubeWiresV(pos, (Vector3){1.0f, TILE_HEIGHT, 1.0f}, DARKGRAY);
        }
      }
    }

    float playerY = groundY + TILE_HEIGHT / 2.0f + 0.01f;
    Vector3 shadowPos = { playerX, playerY, playerZ };
    DrawCircle3D(shadowPos, 0.35f, (Vector3){1, 0, 0}, 90.0f, (Color){0, 0, 0, 100});

    EndMode3D();

    float spriteY = groundY + TILE_HEIGHT + 0.8f;
    Vector3 playerPos3D = { playerX, spriteY, playerZ };
    Vector2 screenPos = GetWorldToScreen(playerPos3D, camera);

    DrawRectangle(screenPos.x - PLAYER_SPRITE_WIDTH/2, screenPos.y - PLAYER_SPRITE_HEIGHT, 
        PLAYER_SPRITE_WIDTH, PLAYER_SPRITE_HEIGHT, RED);
    DrawRectangle(screenPos.x - 12, screenPos.y - PLAYER_SPRITE_HEIGHT + 15, 8, 8, WHITE);
    DrawRectangle(screenPos.x + 4, screenPos.y - PLAYER_SPRITE_HEIGHT + 15, 8, 8, WHITE);
    DrawRectangleLines(screenPos.x - PLAYER_SPRITE_WIDTH/2, screenPos.y - PLAYER_SPRITE_HEIGHT, 
        PLAYER_SPRITE_WIDTH, PLAYER_SPRITE_HEIGHT, BLACK);

    DrawFPS(10, 10);
    DrawText("WASD: Move (Relative) | Q/E: Rotate 45°", 10, 30, 20, WHITE);

    char angleText[50];
    sprintf(angleText, "Camera Angle: %.0f°", cameraAngle * RAD2DEG);
    DrawText(angleText, 10, 55, 20, LIGHTGRAY);

    EndDrawing();
  }

  CloseWindow();
  enet_deinitialize();
  return 0;
}
