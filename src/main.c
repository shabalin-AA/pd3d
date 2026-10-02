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
#define MAP_WIDTH 60
#define MAP_HEIGHT 60
#define FLOOR_HEIGHT 3.0f      // Пол на высоте 3
#define WALL_HEIGHT 7.0f       // Стены на высоте 7
#define MAX_HEIGHT_DIFF 1
#define MAX_PATH_LENGTH 200

// Типы тайлов (высота в world units)
#define TILE_EMPTY 0           // Пропасть
#define TILE_FLOOR 3           // Пол (высота 3)
#define TILE_WALL 7            // Стена (высота 7)

// Типы комнат
#define ROOM_TYPE_NORMAL 0     // Обычная комната со стенами
#define ROOM_TYPE_PILLAR 1     // Комната без стен (платформа)

// Типы коридоров
#define CORRIDOR_TYPE_WALL 0   // Коридор внутри "здания" (вырезан из стен)
#define CORRIDOR_TYPE_BRIDGE 1 // Мост над пропастью

// ==================== КОНСТАНТЫ ГЕНЕРАЦИИ ====================
#define ROOM_GEN_MIN_SIZE 5
#define ROOM_GEN_MAX_SIZE 11
#define ROOM_GEN_COUNT 8          // Увеличили количество комнат
#define ROOM_GEN_MAX_ATTEMPTS 30
#define ROOM_GEN_PADDING 4        // Увеличили отступ между комнатами
#define MAX_ROOMS 20
#define MAX_CONNECTIONS_PER_ROOM 4  // Максимум коридоров на комнату
#define MIN_ENTRANCE_DISTANCE 3    // Минимальное расстояние между входами

// ==================== КОНСТАНТЫ КАМЕРЫ ====================
#define CAMERA_RADIUS 8.0f
#define CAMERA_HEIGHT 16.0f
#define CAMERA_INITIAL_ANGLE (PI / 4.0f)
#define CAMERA_ROTATION_STEP (PI / 4.0f)
#define CAMERA_FOVY_INITIAL 12.0f
#define CAMERA_FOVY_MIN 4.0f    // Максимальный зум (приближение)
#define CAMERA_FOVY_MAX 40.0f   // Максимальное отдаление
#define CAMERA_ZOOM_SPEED 1.0f  // Шаг зума за прокрутку

// ==================== КОНСТАНТЫ ИГРОКА ====================
#define PLAYER_START_X 6.0f
#define PLAYER_START_Z 6.0f
#define PLAYER_MOVE_SPEED 10.0f
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

typedef struct {
  int x, z;
  int width, height;
  int centerX, centerZ;
  int connections;
  Point entrances[MAX_CONNECTIONS_PER_ROOM];
  int type;  // ROOM_TYPE_NORMAL или ROOM_TYPE_PILLAR
} Room;

Room rooms[MAX_ROOMS];
int roomCount = 0;

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
int heightMap[MAP_WIDTH][MAP_HEIGHT] = {0};
Path currentPath = {0};
bool isMoving = false;
float cameraAngle = CAMERA_INITIAL_ANGLE;
float cameraFovy = CAMERA_FOVY_INITIAL;  // Новая переменная
float playerX = PLAYER_START_X;
float playerZ = PLAYER_START_Z;
// Зоны: true = область со стенами (здание), false = пропасть
bool wallZone[MAP_WIDTH][MAP_HEIGHT] = {false};

// ==================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ====================
// ==================== РАССТОЯНИЕ МЕЖДУ КОМНАТАМИ ====================
float distanceBetweenRooms(Room *a, Room *b) {
  int dx = a->centerX - b->centerX;
  int dz = a->centerZ - b->centerZ;
  return sqrtf((float)(dx * dx + dz * dz));
}

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
      if (heightMap[x][z] == TILE_EMPTY) continue;

      // ИСПРАВЛЕНО: heightMap уже содержит высоту
      float tileY = (float)heightMap[x][z];
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

// ==================== ИНИЦИАЛИЗАЦИЯ ЗОН ====================
void initZones() {
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      wallZone[x][z] = false;
    }
  }

  // Область стен в центре карты (40-55% размера)
  int zoneWidth = MAP_WIDTH * 2 / 5 + rand() % (MAP_WIDTH / 6);
  int zoneHeight = MAP_HEIGHT * 2 / 5 + rand() % (MAP_HEIGHT / 6);
  int zoneX = (MAP_WIDTH - zoneWidth) / 2;
  int zoneZ = (MAP_HEIGHT - zoneHeight) / 2;

  for (int x = zoneX; x < zoneX + zoneWidth && x < MAP_WIDTH; x++) {
    for (int z = zoneZ; z < zoneZ + zoneHeight && z < MAP_HEIGHT; z++) {
      wallZone[x][z] = true;
    }
  }
}

// ==================== ИНИЦИАЛИЗАЦИЯ КАРТЫ ====================
void initMap() {
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      heightMap[x][z] = TILE_EMPTY;
    }
  }
}

// ==================== ПОИСК КОМНАТЫ ПО ТОЧКЕ ====================
int findRoomAtPoint(int x, int z) {
  for (int i = 0; i < roomCount; i++) {
    Room *r = &rooms[i];
    if (x >= r->x && x < r->x + r->width &&
        z >= r->z && z < r->z + r->height) {
      return i;
    }
  }
  return -1;
}

// ==================== ПРОВЕРКА ВОЗМОЖНОСТИ ПОДКЛЮЧЕНИЯ ====================
bool canConnectRoom(int roomIndex, Point entrance) {
  Room *r = &rooms[roomIndex];

  // Проверяем лимит подключений
  if (r->connections >= MAX_CONNECTIONS_PER_ROOM) {
    return false;
  }

  // Проверяем расстояние до существующих входов
  for (int i = 0; i < r->connections; i++) {
    int dx = abs(r->entrances[i].x - entrance.x);
    int dz = abs(r->entrances[i].z - entrance.z);
    if (dx + dz < MIN_ENTRANCE_DISTANCE) {
      return false;
    }
  }

  return true;
}

// ==================== РЕГИСТРАЦИЯ ВХОДА В КОМНАТУ ====================
void registerEntrance(int roomIndex, Point entrance) {
  Room *r = &rooms[roomIndex];
  if (r->connections < MAX_CONNECTIONS_PER_ROOM) {
    r->entrances[r->connections] = entrance;
    r->connections++;
  }
}

// ==================== ПРОВЕРКА ПЕРЕСЕЧЕНИЯ КОМНАТ ====================
bool roomOverlaps(Room newRoom) {
  for (int i = 0; i < roomCount; i++) {
    Room r = rooms[i];
    // Проверяем пересечение с учетом отступа
    if (newRoom.x - ROOM_GEN_PADDING < r.x + r.width &&
        newRoom.x + newRoom.width + ROOM_GEN_PADDING > r.x &&
        newRoom.z - ROOM_GEN_PADDING < r.z + r.height &&
        newRoom.z + newRoom.height + ROOM_GEN_PADDING > r.z) {
      return true;
    }
  }
  return false;
}

// ==================== СОЗДАНИЕ КОРИДОРА ====================
void carveCorridor(int x1, int z1, int x2, int z2) {
  if (rand() % 2 == 0) {
    // Горизонтально, потом вертикально
    int stepX = (x2 > x1) ? 1 : -1;
    for (int x = x1; x != x2 + stepX; x += stepX) {
      if (x >= 0 && x < MAP_WIDTH && z1 >= 0 && z1 < MAP_HEIGHT) {
        if (heightMap[x][z1] != TILE_FLOOR) {
          heightMap[x][z1] = TILE_FLOOR;
        }
      }
    }
    int stepZ = (z2 > z1) ? 1 : -1;
    for (int z = z1; z != z2 + stepZ; z += stepZ) {
      if (x2 >= 0 && x2 < MAP_WIDTH && z >= 0 && z < MAP_HEIGHT) {
        if (heightMap[x2][z] != TILE_FLOOR) {
          heightMap[x2][z] = TILE_FLOOR;
        }
      }
    }
  } else {
    // Вертикально, потом горизонтально
    int stepZ = (z2 > z1) ? 1 : -1;
    for (int z = z1; z != z2 + stepZ; z += stepZ) {
      if (x1 >= 0 && x1 < MAP_WIDTH && z >= 0 && z < MAP_HEIGHT) {
        if (heightMap[x1][z] != TILE_FLOOR) {
          heightMap[x1][z] = TILE_FLOOR;
        }
      }
    }
    int stepX = (x2 > x1) ? 1 : -1;
    for (int x = x1; x != x2 + stepX; x += stepX) {
      if (x >= 0 && x < MAP_WIDTH && z2 >= 0 && z2 < MAP_HEIGHT) {
        if (heightMap[x][z2] != TILE_FLOOR) {
          heightMap[x][z2] = TILE_FLOOR;
        }
      }
    }
  }
}

// ==================== ГЕНЕРАЦИЯ ПОДЗЕМЕЛЬЯ ====================
void generateDungeon() {
  initMap();
  initZones();
  roomCount = 0;

  // === ЭТАП 1: Заполняем зоны стенами ===
  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int z = 0; z < MAP_HEIGHT; z++) {
      if (wallZone[x][z]) {
        heightMap[x][z] = TILE_WALL;
      } else {
        heightMap[x][z] = TILE_EMPTY;
      }
    }
  }

  // === ЭТАП 2: Генерация комнат ===
  for (int i = 0; i < ROOM_GEN_COUNT; i++) {
    Room newRoom = {0};
    newRoom.width = ROOM_GEN_MIN_SIZE + rand() % (ROOM_GEN_MAX_SIZE - ROOM_GEN_MIN_SIZE + 1);
    newRoom.height = ROOM_GEN_MIN_SIZE + rand() % (ROOM_GEN_MAX_SIZE - ROOM_GEN_MIN_SIZE + 1);

    // Случайно выбираем тип комнаты
    newRoom.type = (rand() % 2 == 0) ? ROOM_TYPE_NORMAL : ROOM_TYPE_PILLAR;

    if (newRoom.width >= MAP_WIDTH - 8 || newRoom.height >= MAP_HEIGHT - 8) {
      continue;
    }

    bool placed = false;
    for (int attempt = 0; attempt < ROOM_GEN_MAX_ATTEMPTS; attempt++) {
      newRoom.x = 2 + rand() % (MAP_WIDTH - newRoom.width - 4);
      newRoom.z = 2 + rand() % (MAP_HEIGHT - newRoom.height - 4);
      newRoom.centerX = newRoom.x + newRoom.width / 2;
      newRoom.centerZ = newRoom.z + newRoom.height / 2;
      newRoom.connections = 0;

      if (!roomOverlaps(newRoom)) {
        placed = true;
        break;
      }
    }

    if (placed) {
      if (newRoom.type == ROOM_TYPE_NORMAL) {
        // Обычная комната: вырезаем из стен
        for (int x = newRoom.x; x < newRoom.x + newRoom.width; x++) {
          for (int z = newRoom.z; z < newRoom.z + newRoom.height; z++) {
            if (x < 0 || x >= MAP_WIDTH || z < 0 || z >= MAP_HEIGHT) continue;

            if (x == newRoom.x || x == newRoom.x + newRoom.width - 1 ||
                z == newRoom.z || z == newRoom.z + newRoom.height - 1) {
              // Стены остаются
              heightMap[x][z] = TILE_WALL;
            } else {
              // Пол внутри
              heightMap[x][z] = TILE_FLOOR;
            }
          }
        }
      } else {
        // Комната-платформа: просто пол без стен
        for (int x = newRoom.x; x < newRoom.x + newRoom.width; x++) {
          for (int z = newRoom.z; z < newRoom.z + newRoom.height; z++) {
            if (x < 0 || x >= MAP_WIDTH || z < 0 || z >= MAP_HEIGHT) continue;
            heightMap[x][z] = TILE_FLOOR;
          }
        }
      }
      rooms[roomCount++] = newRoom;
    }
  }

  if (roomCount < 2) {
    playerX = MAP_WIDTH / 2.0f;
    playerZ = MAP_HEIGHT / 2.0f;
    return;
  }

  // === ЭТАП 3: MST для связности ===
  bool inMST[MAX_ROOMS] = {false};
  inMST[0] = true;
  int mstCount = 1;

  while (mstCount < roomCount) {
    float bestDist = 999999.0f;
    int bestFrom = -1;
    int bestTo = -1;

    for (int i = 0; i < roomCount; i++) {
      if (!inMST[i]) continue;
      for (int j = 0; j < roomCount; j++) {
        if (inMST[j]) continue;

        float dist = distanceBetweenRooms(&rooms[i], &rooms[j]);
        if (dist < bestDist) {
          bestDist = dist;
          bestFrom = i;
          bestTo = j;
        }
      }
    }

    if (bestFrom == -1 || bestTo == -1) break;

    Room *rA = &rooms[bestFrom];
    Room *rB = &rooms[bestTo];

    if (rA->connections < MAX_CONNECTIONS_PER_ROOM &&
        rB->connections < MAX_CONNECTIONS_PER_ROOM) {
      carveCorridor(rA->centerX, rA->centerZ, rB->centerX, rB->centerZ);
      rA->connections++;
      rB->connections++;
    }

    inMST[bestTo] = true;
    mstCount++;
  }

  // === ЭТАП 4: Дополнительные коридоры ===
  int extraCorridors = roomCount / 2;
  for (int attempt = 0; attempt < extraCorridors * 3 && extraCorridors > 0; attempt++) {
    int roomA = rand() % roomCount;
    int roomB = rand() % roomCount;

    if (roomA == roomB) continue;

    Room *rA = &rooms[roomA];
    Room *rB = &rooms[roomB];

    if (rA->connections < MAX_CONNECTIONS_PER_ROOM &&
        rB->connections < MAX_CONNECTIONS_PER_ROOM) {
      carveCorridor(rA->centerX, rA->centerZ, rB->centerX, rB->centerZ);
      rA->connections++;
      rB->connections++;
      extraCorridors--;
    }
  }

  playerX = (float)rooms[0].centerX;
  playerZ = (float)rooms[0].centerZ;
}

// ==================== ОБНОВЛЕНИЕ КАМЕРЫ ====================
void updateCamera(Camera3D *camera) {
  int gridX = (int)roundf(playerX);
  int gridZ = (int)roundf(playerZ);

  // ИСПРАВЛЕНО: heightMap уже содержит высоту (3 или 7), умножать не нужно
  float groundY = (float)heightMap[gridX][gridZ];

  camera->target = (Vector3){ playerX, groundY + 0.5f, playerZ };
  camera->position = (Vector3){
    camera->target.x + CAMERA_RADIUS * cosf(cameraAngle),
    camera->target.y + CAMERA_HEIGHT,
    camera->target.z + CAMERA_RADIUS * sinf(cameraAngle)
  };
  camera->fovy = cameraFovy;
}

// ==================== ОБРАБОТКА ВВОДА ====================
void handleInput(Camera3D camera) {
  if (IsKeyPressed(KEY_Q)) {
    cameraAngle -= CAMERA_ROTATION_STEP;
  }
  if (IsKeyPressed(KEY_E)) {
    cameraAngle += CAMERA_ROTATION_STEP;
  }

  // Зум колесиком мыши
  float wheel = GetMouseWheelMove();
  if (wheel != 0.0f) {
    cameraFovy -= wheel * CAMERA_ZOOM_SPEED;
    // Ограничиваем диапазон
    if (cameraFovy < CAMERA_FOVY_MIN) cameraFovy = CAMERA_FOVY_MIN;
    if (cameraFovy > CAMERA_FOVY_MAX) cameraFovy = CAMERA_FOVY_MAX;
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
        // ИСПРАВЛЕНО: высота тайла равна его числовому значению (3 или 7)
        float columnHeight = (float)tileType;
        Vector3 size = { 1.0f, columnHeight, 1.0f };
        Vector3 pos = { (float)x, columnHeight / 2.0f, (float)z };

        Color tileColor;
        if (tileType == TILE_WALL) {
          tileColor = (Color){ 50, 50, 55, 255 };       // Тёмные стены
        } else if (wallZone[x][z]) {
          tileColor = (Color){ 100, 100, 105, 255 };    // Серый камень (здание)
        } else {
          tileColor = (Color){ 130, 95, 60, 255 };      // Коричневое дерево (мост)
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

    // ИСПРАВЛЕНО: heightMap уже содержит высоту
    float tileY = (float)heightMap[p.x][p.z] + 0.1f;
    Vector3 pathPos = {(float)p.x, tileY, (float)p.z};
    DrawCubeV(pathPos, (Vector3){0.3f, 0.05f, 0.3f}, YELLOW);
  }
}

// ==================== РЕНДЕР ТЕНИ ИГРОКА (3D) ====================
void renderPlayerShadow() {
  int gridX = (int)roundf(playerX);
  int gridZ = (int)roundf(playerZ);

  // ИСПРАВЛЕНО
  float groundY = (float)heightMap[gridX][gridZ];

  Vector3 shadowPos = { playerX, groundY + 0.01f, playerZ };
  DrawCircle3D(shadowPos, 0.35f, (Vector3){1, 0, 0}, 90.0f, (Color){0, 0, 0, 100});
}

// ==================== РЕНДЕР СПРАЙТА ИГРОКА (2D) ====================
void renderPlayerSprite(Camera3D camera) {
  int gridX = (int)roundf(playerX);
  int gridZ = (int)roundf(playerZ);

  // ИСПРАВЛЕНО
  float groundY = (float)heightMap[gridX][gridZ];

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
  camera.fovy = cameraFovy;
  camera.projection = CAMERA_ORTHOGRAPHIC;

  generateDungeon();

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
