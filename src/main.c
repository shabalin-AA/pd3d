#define ENET_IMPLEMENTATION
#include "enet.h"
#include "raylib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Размер карты
#define MAP_WIDTH 12
#define MAP_HEIGHT 12

// Высота тайла (толщина плиты)
#define TILE_HEIGHT 0.2f
// Шаг высоты (насколько высоко поднимается каждая ступенька)
#define HEIGHT_STEP 1.0f 

int main(void)
{
  if (enet_initialize() != 0) {
    printf("An error occurred while initializing ENet.\n");
    return 1;
  }

  InitWindow(1024, 768, "Isometric Pixel Dungeon - Prototype");
  SetTargetFPS(60);

  // 1. Настройка Изометрической Камеры
  Camera3D camera = {0};
  camera.position = (Vector3){ 15.0f, 12.0f, 15.0f }; 
  camera.target = (Vector3){ 6.0f, 0.0f, 6.0f }; 
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  camera.fovy = 12.0f; // Для ортографической камеры это масштаб (зум)
  camera.projection = CAMERA_ORTHOGRAPHIC; 

  // 2. Генерация тестовой карты высот
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

  // 3. Состояние игрока
  float targetX = 6.0f; 
  float targetZ = 6.0f;
  float playerX = 6.0f;
  float playerZ = 6.0f;

  float moveSpeed = 6.0f; // Клеток в секунду

  while (!WindowShouldClose())
  {
    // --- UPDATE ---
    float dt = GetFrameTime();

    // Обработка ввода (Пошаговая логика)
    if (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_W)) targetZ -= 1.0f;
    if (IsKeyPressed(KEY_DOWN) || IsKeyPressed(KEY_S)) targetZ += 1.0f;
    if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_A)) targetX -= 1.0f;
    if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) targetX += 1.0f;

    // Проверка границ карты и "пропастей" (высота 0)
    int intTargetX = (int)roundf(targetX);
    int intTargetZ = (int)roundf(targetZ);

    if (intTargetX < 0 || intTargetX >= MAP_WIDTH || 
        intTargetZ < 0 || intTargetZ >= MAP_HEIGHT || 
        heightMap[intTargetX][intTargetZ] == 0) {
      // Откатываем цель к текущей позиции, если ход недопустим
      targetX = playerX; 
      targetZ = playerZ;
    }

    // Плавная интерполяция визуальной позиции к целевой
    float step = moveSpeed * dt;
    float dx = targetX - playerX;
    float dz = targetZ - playerZ;

    if (fabsf(dx) > 0.01f) playerX += (dx > 0 ? 1.0f : -1.0f) * fminf(step, fabsf(dx));
    else playerX = targetX;

    if (fabsf(dz) > 0.01f) playerZ += (dz > 0 ? 1.0f : -1.0f) * fminf(step, fabsf(dz));
    else playerZ = targetZ;


    // --- DRAW ---
    BeginDrawing();
    ClearBackground((Color){30, 30, 40, 255}); 

    BeginMode3D(camera);

    // Рендер 3D-тайлов (Пола)
    for (int x = 0; x < MAP_WIDTH; x++) {
      for (int z = 0; z < MAP_HEIGHT; z++) {
        int h = heightMap[x][z];
        if (h > 0) {
          Vector3 pos = { 
            (float)x, 
            (float)h * HEIGHT_STEP, 
            (float)z 
          };

          unsigned char brightness = 80 + h * 30;
          Color tileColor = { brightness, brightness, brightness + 20, 255 };

          DrawCubeV(pos, (Vector3){1.0f, TILE_HEIGHT, 1.0f}, tileColor);
          DrawCubeWiresV(pos, (Vector3){1.0f, TILE_HEIGHT, 1.0f}, DARKGRAY);
        }
      }
    }

    // Рендер "Тени" игрока в 3D (для привязки к полу)
    int gridX = (int)roundf(playerX);
    int gridZ = (int)roundf(playerZ);
    float playerY = (float)heightMap[gridX][gridZ] * HEIGHT_STEP + TILE_HEIGHT / 2.0f + 0.01f;
    Vector3 shadowPos = { playerX, playerY, playerZ };
    DrawCircle3D(shadowPos, 0.35f, (Vector3){1, 0, 0}, 90.0f, (Color){0, 0, 0, 100});

    EndMode3D();

    // --- РЕНДЕР 2D СПРАЙТА ПОВЕРХ 3D ---
    // Вычисляем 3D позицию центра/головы спрайта
    float spriteY = (float)heightMap[gridX][gridZ] * HEIGHT_STEP + TILE_HEIGHT + 0.8f;
    Vector3 playerPos3D = { playerX, spriteY, playerZ };

    // ИСПРАВЛЕНО: Используем GetWorldToScreen для 3D -> 2D проекции
    Vector2 screenPos = GetWorldToScreen(playerPos3D, camera);

    // Рисуем 2D спрайт (прямо на экране, без BeginMode2D)
    int spriteW = 32;
    int spriteH = 48;

    // Тело
    DrawRectangle(screenPos.x - spriteW/2, screenPos.y - spriteH, spriteW, spriteH, RED);
    // "Глаза"
    DrawRectangle(screenPos.x - 8, screenPos.y - spriteH + 10, 6, 6, WHITE);
    DrawRectangle(screenPos.x + 2, screenPos.y - spriteH + 10, 6, 6, WHITE);

    // Обводка спрайта
    DrawRectangleLines(screenPos.x - spriteW/2, screenPos.y - spriteH, spriteW, spriteH, BLACK);

    // UI
    DrawFPS(10, 10);
    DrawText("WASD to move (Turn-based logic, smooth visual)", 10, 30, 20, WHITE);

    EndDrawing();
  }

  CloseWindow();
  enet_deinitialize();
  return 0;
}
