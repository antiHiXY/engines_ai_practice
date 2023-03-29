#include "raylib.h"
#include <functional>
#include <vector>
#include <limits>
#include <cmath>
#include "math.h"
#include "dungeonGen.h"
#include "dungeonUtils.h"

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

static void draw_nav_grid(const char *input, size_t width, size_t height)
{
  for (size_t y = 0; y < height; ++y)
    for (size_t x = 0; x < width; ++x)
    {
      char symb = input[coord_to_idx(x, y, width)];
      Color color = GetColor(symb == ' ' ? 0xeeeeeeff : symb == 'o' ? 0x7777ffff : 0x222222ff);
      DrawPixel(int(x), int(y), color);
    }
}

static void draw_path(std::vector<Position> path)
{
  for (const Position &p : path)
    DrawPixel(p.x, p.y, GetColor(0x44000088));
}

static std::vector<Position> reconstruct_path(std::vector<Position> prev, Position to, size_t width)
{
  Position curPos = to;
  std::vector<Position> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != Position{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

static std::vector<Position> find_path_a_star(const char *input, size_t width, size_t height, Position from, Position to)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
    return std::vector<Position>();
  size_t inpSize = width * height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<Position> prev(inpSize, {-1,-1});

  auto getG = [&](Position p) -> float { return g[coord_to_idx(p.x, p.y, width)]; };
  auto getF = [&](Position p) -> float { return f[coord_to_idx(p.x, p.y, width)]; };

  auto heuristic = [](Position lhs, Position rhs) -> float
  {
    return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
  };

  g[coord_to_idx(from.x, from.y, width)] = 0;
  f[coord_to_idx(from.x, from.y, width)] = heuristic(from, to);

  std::vector<Position> openList = {from};
  std::vector<Position> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, width);
    Position curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, width);
    DrawPixel(curPos.x, curPos.y, Color{uint8_t(g[idx]), uint8_t(g[idx]), 0, 100});
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](Position p)
    {
      // out of bounds
      if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
        return;
      size_t idx = coord_to_idx(p.x, p.y, width);
      // not empty
      if (input[idx] == '#')
        return;
      float weight = input[idx] == 'o' ? 10.f : 1.f;
      float gScore = getG(curPos) + 1.f * weight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<Position>();
}

std::vector<Position> prevPath;
std::vector<Position> currentPathOnDraw;
static std::vector<float> gScoreAra;
static std::vector<Position> openListAra = {};
std::vector<Position> nextIter;

static std::vector<Position> find_path_ara_star(const char* input, size_t width, size_t height, Position from, Position to, float epsilon)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
    return std::vector<Position>();

  auto getG = [&](Position p) -> float { return gScoreAra[coord_to_idx(p.x, p.y, width)]; };

  auto heuristic = [](Position lhs, Position rhs) -> float
  {
    return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
  };

  auto getF = [&](Position p) -> float
  {
    return getG(p) + epsilon * heuristic(p, to);
  };

  std::vector<Position> closedList;

  while (!openListAra.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openListAra[0]);
    for (size_t i = 1; i < openListAra.size(); ++i)
    {
      float score = getF(openListAra[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (getF(to) > getF(openListAra[bestIdx]))
    {
      Position curPos = openListAra[bestIdx];
      currentPathOnDraw.push_back(curPos);
      openListAra.erase(openListAra.begin() + bestIdx);
      closedList.emplace_back(curPos);
      auto checkNeighbour = [&](Position p)
      {
        // out of bounds
        if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
          return;
        size_t idx = coord_to_idx(p.x, p.y, width);
        // not empty
        if (input[idx] == '#')
          return;
        float weight = input[idx] == 'o' ? 10.f : 1.f;
        float gScore = getG(curPos) + 1.f * weight; // we're exactly 1 unit away
        if (gScore < getG(p))
        {
          prevPath[idx] = curPos;
          gScoreAra[idx] = gScore;
          bool found = std::find(closedList.begin(), closedList.end(), p) != closedList.end();
          if (!found)
          {
            bool foundAra = std::find(openListAra.begin(), openListAra.end(), p) != openListAra.end();
            if (!foundAra)
              openListAra.emplace_back(p);
          }
          else
          {
            bool found = std::find(nextIter.begin(), nextIter.end(), p) != nextIter.end();
            if (!found)
              nextIter.emplace_back(p);
          }
        }
      };
      checkNeighbour({ curPos.x + 1, curPos.y + 0 });
      checkNeighbour({ curPos.x - 1, curPos.y + 0 });
      checkNeighbour({ curPos.x + 0, curPos.y + 1 });
      checkNeighbour({ curPos.x + 0, curPos.y - 1 });
    }
    else
    {
      for (int i = 0; i < nextIter.size(); ++i)
      {
        bool found = std::find(openListAra.begin(), openListAra.end(), nextIter[i]) != openListAra.end();
        if (!found)
          openListAra.emplace_back(nextIter[i]);
      }
      nextIter.clear();
      return reconstruct_path(prevPath, to, width);
    }
  }
  return std::vector<Position>();
}

constexpr float eps_def = 5.0f;
constexpr float eps_step = 0.5f;
static float eps = eps_def;
int next = 0;
static std::vector<Position> lastPath;

void draw_nav_sma_data(const char* input, size_t width, size_t height, Position from, Position to)
{
  draw_nav_grid(input, width, height);
  std::vector<Position> path = find_path_a_star(input, width, height, from, to);
  draw_path(path);
}

void draw_nav_ara_data(const char* input, size_t width, size_t height, Position from, Position to, int& next)
{
  draw_nav_grid(input, width, height);
  --next;
  if (next < 0)
  {
    if (eps < 1.f || openListAra.empty())
    {
      eps = eps_def;
      openListAra = { from };
      size_t inpSize = width * height;
      gScoreAra.clear();
      gScoreAra.resize(inpSize, std::numeric_limits<float>::max());
      gScoreAra[coord_to_idx(from.x, from.y, width)] = 0;
      prevPath.clear();
      prevPath.resize(inpSize, { -1,-1 });
    }
    next = 30;
    eps -= eps_step;
    currentPathOnDraw.clear();
    lastPath = find_path_ara_star(input, width, height, from, to, eps);
  }
  for (int i = 0; i < currentPathOnDraw.size(); ++i)
  {
    size_t idx = coord_to_idx(currentPathOnDraw[i].x, currentPathOnDraw[i].y, width);
    DrawPixel(currentPathOnDraw[i].x, currentPathOnDraw[i].y, Color{ uint8_t(gScoreAra[idx]), uint8_t(gScoreAra[idx]), 0, 100 });
  }
  draw_path(lastPath);
}

int main(int /*argc*/, const char ** /*argv*/)
{
  int width = 1920;
  int height = 1080;
  InitWindow(width, height, "w3 AI MIPT");

  const int scrWidth = GetMonitorWidth(0);
  const int scrHeight = GetMonitorHeight(0);
  if (scrWidth < width || scrHeight < height)
  {
    width = std::min(scrWidth, width);
    height = std::min(scrHeight - 150, height);
    SetWindowSize(width, height);
  }

  constexpr size_t dungWidth = 100;
  constexpr size_t dungHeight = 100;
  char *navGrid = new char[dungWidth * dungHeight];
  gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
  spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);

  Position from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
  Position to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);

  Camera2D camera = { {0, 0}, {0, 0}, 0.f, 1.f };
  //camera.offset = Vector2{ width * 0.5f, height * 0.5f };
  camera.zoom = float(height) / float(dungHeight);

  SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
  while (!WindowShouldClose())
  {
    // pick pos
    Vector2 mousePosition = GetScreenToWorld2D(GetMousePosition(), camera);
    Position p{int(mousePosition.x), int(mousePosition.y)};
    if (IsMouseButtonPressed(2) || IsKeyPressed(KEY_Q))
    {
      size_t idx = coord_to_idx(p.x, p.y, dungWidth);
      if (idx < dungWidth * dungHeight)
        navGrid[idx] = navGrid[idx] == ' ' ? '#' : navGrid[idx] == '#' ? 'o' : ' ';
    }
    else if (IsMouseButtonPressed(0))
    {
      Position &target = from;
      target = p;
      eps = eps_def;
      openListAra = { from };
      size_t inpSize = dungWidth * dungHeight;
      gScoreAra.clear();
      gScoreAra.resize(inpSize, std::numeric_limits<float>::max());
      gScoreAra[coord_to_idx(from.x, from.y, dungWidth)] = 0;
      prevPath.clear();
      prevPath.resize(inpSize, { -1,-1 });
    }
    else if (IsMouseButtonPressed(1))
    {
      Position &target = to;
      target = p;
      eps = eps_def;
      openListAra = { from };
      size_t inpSize = dungWidth * dungHeight;
      gScoreAra.clear();
      gScoreAra.resize(inpSize, std::numeric_limits<float>::max());
      gScoreAra[coord_to_idx(from.x, from.y, dungWidth)] = 0;
      prevPath.clear();
      prevPath.resize(inpSize, { -1,-1 });
    }
    if (IsKeyPressed(KEY_SPACE))
    {
      gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
      spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);
      from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
      to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
    }
    BeginDrawing();
      ClearBackground(BLACK);
      BeginMode2D(camera);
      draw_nav_ara_data(navGrid, dungWidth, dungHeight, from, to, next);
      EndMode2D();
    EndDrawing();
  }
  CloseWindow();
  return 0;
}
