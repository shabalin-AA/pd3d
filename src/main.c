#define ENET_IMPLEMENTATION
#include "enet.h"
#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

// ==================== КОНСТАНТЫ КАРТЫ ====================
#define MAP_WIDTH 12
#define MAP_HEIGHT 12
#define FLOOR_HEIGHT 1.0f
#define WALL_HEIGHT 4.0f
#define MAX_HEIGHT_DIFF 1
#define MAX_PATH_LENGTH 100
#define HEIGHT_STEP 1.0f

// Типы тайлов
#define TILE_EMPTY 0
#define TILE_FLOOR 1
#define TILE_WALL 4

// ==================== КОНСТАНТЫ КАМЕРЫ ====================
#define CAMERA_RADIUS 8.0f
#define CAMERA_HEIGHT 16.0f
#define CAMERA_INITIAL_ANGLE (PI / 4.0f)
#define CAMERA_ROTATION_STEP (PI / 4.0f)
#define CAMERA_FOVY 12.0f

// ==================== КОНСТАНТЫ ИГРОКА ====================
#define PLAYER_START_X 6.0f
#define PLAYER_START_Z 6.0f
#define PLAYER_MOVE_SPEED 4.0f
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
  DIR_COUNT
} Direction;

typedef struct {
  int dx;
  int dz;
} Dir2D;

Dir2D baseDirections[DIR_COUNT] = {
  {-1, -1}, { 0, -1}, { 1, -1}, { 1,  0},
  { 1,  1}, { 0,  1}, {-1,  1}, {-1,  0}
};

// ==================== СТРУКТУРЫ ДЛЯ A* ====================
typedef struct {
  int x;
  int z;
} Point;

typedef struct {
  Point point;
  float g;
  float h;
  float f;
} AStarNode;

typedef struct {
  Point path[MAX_PATH_LENGTH];
  int length;
  int currentIndex;
} Path;

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
int heightMap[MAP_WIDTH][MAP_HEIGHT] = {0};
Path currentPath = {0};
bool isMoving = false;
float cameraAngle = CAMERA_INITIAL_ANGLE;
float playerX = PLAYER_START_X;
float playerZ = PLAYER_START_Z;

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================
float heuristic(Point a, Point b) {
  int dx = abs(a.x - b.x);
  int dz = abs(a.z - b.z);
  return (float)(dx + dz);
}

bool isWalkable(int x, int z, int fromHeight) {
  if (x < 0 || x >= MAP_WIDTH || z < 0 || z >= MAP_HEIGHT) return false;
  if (heightMap[x][z] == 0) return false;
  if (abs(heightMap[x][z] - fromHeight) > MAX_HEIGHT_DIFF) return false;
  return true;
}

// A* алгоритм (исправленная версия)
Path findPath(Point start, Point goal) {
  Path path = {0};

  // Массив стоимости от старта
  float gScore[MAP_WIDTH][MAP_HEIGHT];
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      gScore[x][z] = 999999.0f;
    }
  }
  gScore[start.x][start.z] = 0.0f;

  // Массив для хранения родителя
  Point cameFrom[MAP_WIDTH][MAP_HEIGHT];
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      cameFrom[x][z] = (Point){-1, -1};
    }
  }

  // Открытый список
  AStarNode openList[MAP_WIDTH * MAP_HEIGHT];
  int openCount = 0;

  // Закрытый список
  bool closedList[MAP_WIDTH][MAP_HEIGHT] = {false};

  // Добавляем стартовую точку
  AStarNode startNode = {start, 0.0f, heuristic(start, goal), 0.0f};
  startNode.f = startNode.g + startNode.h;
  openList[openCount++] = startNode;

  while (openCount > 0) {
    // Находим узел с наименьшим F
    int currentIndex = 0;
    for (int i = 1; i < openCount; i++) {
      if (openList[i].f < openList[currentIndex].f) {
        currentIndex = i;
      }
    }

    AStarNode current = openList[currentIndex];

    // Достигли цели
    if (current.point.x == goal.x && current.point.z == goal.z) {
      // Восстанавливаем путь
      Point tempPath[MAX_PATH_LENGTH];
      int tempLength = 0;

      Point p = current.point;
      while (!(p.x == start.x && p.z == start.z) && tempLength < MAX_PATH_LENGTH) {
        tempPath[tempLength++] = p;
        p = cameFrom[p.x][p.z];
        if (p.x == -1) break;  // Защита от бесконечного цикла
      }

      // Инвертируем путь
      path.length = tempLength;
      for (int i = 0; i < tempLength; i++) {
        path.path[i] = tempPath[tempLength - 1 - i];
      }

      return path;
    }

    // Перемещаем в закрытый список
    openList[currentIndex] = openList[--openCount];
    closedList[current.point.x][current.point.z] = true;

    // Проверяем соседей
    for (int dir = 0; dir < DIR_COUNT; dir++) {
      Point neighbor = {
        current.point.x + baseDirections[dir].dx,
        current.point.z + baseDirections[dir].dz
      };

      if (!isWalkable(neighbor.x, neighbor.z, heightMap[current.point.x][current.point.z])) {
        continue;
      }

      if (closedList[neighbor.x][neighbor.z]) {
        continue;
      }

      float tentativeG = current.g + 1.0f;

      if (tentativeG < gScore[neighbor.x][neighbor.z]) {
        cameFrom[neighbor.x][neighbor.z] = current.point;
        gScore[neighbor.x][neighbor.z] = tentativeG;

        // Проверяем, есть ли уже в открытом списке
        bool inOpen = false;
        for (int i = 0; i < openCount; i++) {
          if (openList[i].point.x == neighbor.x && openList[i].point.z == neighbor.z) {
            openList[i].g = tentativeG;
            openList[i].f = tentativeG + openList[i].h;
            inOpen = true;
            break;
          }
        }

        if (!inOpen) {
          AStarNode newNode = {neighbor, tentativeG, heuristic(neighbor, goal), 0.0f};
          newNode.f = newNode.g + newNode.h;
          openList[openCount++] = newNode;
        }
      }
    }
  }

  return path;
}

Point screenToGrid(Vector2 screenPos, Camera3D camera) {
  Point result = {-1, -1};
  float minDist = 50.0f;

  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      if (heightMap[x][z] == 0) continue;

      float tileY = (float)heightMap[x][z] * HEIGHT_STEP;
      Vector3 tilePos = {(float)x, tileY, (float)z};
      Vector2 tileScreen = GetWorldToScreen(tilePos, camera);

      float dist = Vector2Distance(screenPos, tileScreen);
      if (dist < minDist) {
        minDist = dist;
        result.x = x;
        result.z = z;
      }
    }
  }

  return result;
}

// ==================== ИНИЦИАЛИЗАЦИЯ КАРТЫ ====================
void initMap() {
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      heightMap[x][z] = TILE_EMPTY;
    }
  }
}

// ==================== ГЕНЕРАЦИЯ КОМНАТЫ ====================
void generateRoom() {
  initMap();

  int roomWidth = 5 + rand() % 7;
  int roomHeight = 5 + rand() % 7;

  int startX = (MAP_WIDTH - roomWidth) / 2;
  int startZ = (MAP_HEIGHT - roomHeight) / 2;

  for (int x = startX; x < startX + roomWidth; x++) {
    for (int z = startZ; z < startZ + roomHeight; z++) {
      if (x == startX || x == startX + roomWidth - 1 ||
          z == startZ || z == startZ + roomHeight - 1) {
        heightMap[x][z] = TILE_WALL;
      } else {
        heightMap[x][z] = TILE_FLOOR;
      }
    }
  }

  // Игрок появляется в центре комнаты на полу
  playerX = startX + roomWidth / 2.0f;
  playerZ = startZ + roomHeight / 2.0f;
}

// ==================== ОБНОВЛЕНИЕ КАМЕРЫ ====================
void updateCamera(Camera3D *camera) {
  int gridX = (int)roundf(playerX);
  int gridZ = (int)roundf(playerZ);
  float groundY = (float)heightMap[gridX][gridZ] * FLOOR_HEIGHT;

  camera->target = (Vector3){ playerX, groundY + 0.5f, playerZ };
  camera->position = (Vector3){
    camera->target.x + CAMERA_RADIUS * cosf(cameraAngle),
    camera->target.y + CAMERA_HEIGHT,
    camera->target.z + CAMERA_RADIUS * sinf(cameraAngle)
  };
}

// ==================== ОБРАБОТКА ВВОДА ====================
void handleInput(Camera3D camera) {
  if (IsKeyPressed(KEY_Q)) {
    cameraAngle -= CAMERA_ROTATION_STEP;
  }
  if (IsKeyPressed(KEY_E)) {
    cameraAngle += CAMERA_ROTATION_STEP;
  }

  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    Vector2 mousePos = GetMousePosition();
    Point target = screenToGrid(mousePos, camera);

    if (target.x >= 0 && target.z >= 0) {
      Point start = {(int)roundf(playerX), (int)roundf(playerZ)};
      currentPath = findPath(start, target);
      currentPath.currentIndex = 0;
      isMoving = (currentPath.length > 0);
    }
  }
}

// ==================== ОБНОВЛЕНИЕ ДВИЖЕНИЯ ====================
void updateMovement() {
  if (!isMoving || currentPath.currentIndex >= currentPath.length) {
    isMoving = false;
    return;
  }

  float dt = GetFrameTime();
  Point nextPoint = currentPath.path[currentPath.currentIndex];
  float targetX = (float)nextPoint.x;
  float targetZ = (float)nextPoint.z;

  float step = PLAYER_MOVE_SPEED * dt;
  float dx = targetX - playerX;
  float dz = targetZ - playerZ;

  if (fabsf(dx) > 0.01f) playerX += (dx > 0 ? 1.0f : -1.0f) * fminf(step, fabsf(dx));
  else playerX = targetX;

  if (fabsf(dz) > 0.01f) playerZ += (dz > 0 ? 1.0f : -1.0f) * fminf(step, fabsf(dz));
  else playerZ = targetZ;

  if (fabsf(playerX - targetX) < 0.01f && fabsf(playerZ - targetZ) < 0.01f) {
    currentPath.currentIndex++;
    if (currentPath.currentIndex >= currentPath.length) {
      isMoving = false;
    }
  }
}

// ==================== РЕНДЕР ТАЙЛОВ ====================
void renderTiles() {
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      int tileType = heightMap[x][z];
      if (tileType > 0) {
        float columnHeight = (tileType == TILE_WALL) ? WALL_HEIGHT : FLOOR_HEIGHT;
        Vector3 size = { 1.0f, columnHeight, 1.0f };
        Vector3 pos = { (float)x, columnHeight / 2.0f, (float)z };

        Color tileColor;
        if (tileType == TILE_WALL) {
          tileColor = (Color){ 60, 60, 70, 255 };
        } else {
          tileColor = (Color){ 120, 110, 100, 255 };
        }

        DrawCubeV(pos, size, tileColor);
        DrawCubeWiresV(pos, size, DARKGRAY);
      }
    }
  }
}

// ==================== РЕНДЕР ПУТИ ====================
void renderPath() {
  if (!isMoving) return;

  for (int i = currentPath.currentIndex; i < currentPath.length; i++) {
    Point p = currentPath.path[i];
    float tileY = (float)heightMap[p.x][p.z] * FLOOR_HEIGHT + 0.1f;
    Vector3 pathPos = {(float)p.x, tileY, (float)p.z};
    DrawCubeV(pathPos, (Vector3){0.3f, 0.05f, 0.3f}, YELLOW);
  }
}

// ==================== РЕНДЕР ТЕНИ ИГРОКА (3D) ====================
void renderPlayerShadow() {
  int gridX = (int)roundf(playerX);
  int gridZ = (int)roundf(playerZ);
  float groundY = (float)heightMap[gridX][gridZ] * FLOOR_HEIGHT;

  Vector3 shadowPos = { playerX, groundY + 0.01f, playerZ };
  DrawCircle3D(shadowPos, 0.35f, (Vector3){1, 0, 0}, 90.0f, (Color){0, 0, 0, 100});
}

// ==================== РЕНДЕР СПРАЙТА ИГРОКА (2D) ====================
void renderPlayerSprite(Camera3D camera) {
  int gridX = (int)roundf(playerX);
  int gridZ = (int)roundf(playerZ);
  float groundY = (float)heightMap[gridX][gridZ] * FLOOR_HEIGHT;

  float spriteY = groundY + 0.8f;
  Vector3 playerPos3D = { playerX, spriteY, playerZ };
  Vector2 screenPos = GetWorldToScreen(playerPos3D, camera);

  DrawRectangle(screenPos.x - PLAYER_SPRITE_WIDTH/2, screenPos.y - PLAYER_SPRITE_HEIGHT,
                PLAYER_SPRITE_WIDTH, PLAYER_SPRITE_HEIGHT, RED);
  DrawRectangle(screenPos.x - 12, screenPos.y - PLAYER_SPRITE_HEIGHT + 15, 8, 8, WHITE);
  DrawRectangle(screenPos.x + 4, screenPos.y - PLAYER_SPRITE_HEIGHT + 15, 8, 8, WHITE);
  DrawRectangleLines(screenPos.x - PLAYER_SPRITE_WIDTH/2, screenPos.y - PLAYER_SPRITE_HEIGHT,
                    PLAYER_SPRITE_WIDTH, PLAYER_SPRITE_HEIGHT, BLACK);
}

// ==================== РЕНДЕР UI ====================
void renderUI() {
  DrawFPS(10, 10);
  DrawText("Click to move | Q/E: Rotate Camera", 10, 30, 20, WHITE);

  char angleText[50];
  sprintf(angleText, "Camera Angle: %.0f°", cameraAngle * RAD2DEG);
  DrawText(angleText, 10, 55, 20, LIGHTGRAY);

  char pathText[50];
  sprintf(pathText, "Path length: %d", currentPath.length);
  DrawText(pathText, 10, 80, 20, LIGHTGRAY);
}

// ==================== MAIN ====================
int main(void)
{
  // Инициализация генератора случайных чисел
  srand(time(NULL));

  if (enet_initialize() != 0) {
    printf("An error occurred while initializing ENet.\n");
    return 1;
  }

  InitWindow(1600, 900, "Isometric Pixel Dungeon - A* Pathfinding");
  SetTargetFPS(60);

  Camera3D camera = {0};
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  camera.fovy = CAMERA_FOVY;
  camera.projection = CAMERA_ORTHOGRAPHIC;

  generateRoom();

  while (!WindowShouldClose()) {
    handleInput(camera);
    updateMovement();
    updateCamera(&camera);

    BeginDrawing();
    ClearBackground((Color){30, 30, 40, 255});

    BeginMode3D(camera);
    renderTiles();
    renderPath();
    renderPlayerShadow();  // Тень в 3D
    EndMode3D();

    renderPlayerSprite(camera);  // Спрайт в 2D после EndMode3D
    renderUI();

    EndDrawing();
  }

  CloseWindow();
  enet_deinitialize();
  return 0;
}
