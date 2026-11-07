#include "pathfinder.h"

extern uint8_t field[pathfinder::kFieldWidth][pathfinder::kFieldHeight];
extern uint8_t Ball[32][32];
extern uint8_t robot[32][32];

namespace pathfinder
{
namespace
{
uint8_t fieldCopy[kFieldWidth][kFieldHeight] = {0};
uint16_t gScore[kFieldWidth][kFieldHeight];
uint16_t gStamp[kFieldWidth][kFieldHeight] = {0};
uint16_t closedStamp[kFieldWidth][kFieldHeight] = {0};
uint16_t searchStamp = 1;

uint8_t heapX[6000];
uint8_t heapY[6000];
uint16_t heapG[6000];
uint32_t heapF[6000];
uint16_t heapSize = 0;

uint8_t origin[kFieldWidth][kFieldHeight] = {0};
const int8_t dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
const int8_t dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

uint8_t currentStart[2] = {0, 0};
uint8_t currentTarget[2] = {0, 0};
uint16_t currentnode[5] = {0};
bool pathFound = true;
bool fieldCopyInitialized = false;

inline uint16_t getTargetDistance(uint16_t x, uint16_t y)
{
	uint16_t xdist = (x > currentTarget[0]) ? (x - currentTarget[0]) : (currentTarget[0] - x);
	uint16_t ydist = (y > currentTarget[1]) ? (y - currentTarget[1]) : (currentTarget[1] - y);
	uint16_t diagonalDistance = (xdist < ydist) ? xdist : ydist;
	uint16_t straightDistance = (xdist > ydist) ? (xdist - ydist) : (ydist - xdist);

	return diagonalDistance * 14 + straightDistance * 10;
}

inline uint16_t getKnownGScore(uint16_t x, uint16_t y)
{
	return (gStamp[x][y] == searchStamp) ? gScore[x][y] : 0xFFFF;
}

inline void setKnownGScore(uint16_t x, uint16_t y, uint16_t g)
{
	gStamp[x][y] = searchStamp;
	gScore[x][y] = g;
}

inline bool isClosed(uint16_t x, uint16_t y)
{
	return closedStamp[x][y] == searchStamp;
}

void ensureFieldCopyInitialized()
{
	if (fieldCopyInitialized)
		return;

	for (uint16_t x = 0; x < kFieldWidth; x++)
		for (uint16_t y = 0; y < kFieldHeight; y++)
			fieldCopy[x][y] = field[x][y];

	fieldCopyInitialized = true;
}

void restoreField()
{
	for (uint16_t x = 0; x < kFieldWidth; x++)
		for (uint16_t y = 0; y < kFieldHeight; y++)
			field[x][y] = fieldCopy[x][y];
}

template <size_t Size>
void placeCenteredMask(uint8_t (&mask)[Size][Size], uint8_t centerX, uint8_t centerY)
{
	const int halfSize = int(Size / 2);

	for (int maskX = 0; maskX < int(Size); maskX++)
	{
		for (int maskY = 0; maskY < int(Size); maskY++)
		{
			int fieldX = int(centerX) + maskX - halfSize;
			int fieldY = int(centerY) + maskY - halfSize;

			if (fieldX < 0 || fieldX >= kFieldWidth || fieldY < 0 || fieldY >= kFieldHeight)
				continue;

			field[fieldX][fieldY] = mask[maskX][maskY];
		}
	}
}

void pushHeap(uint8_t x, uint8_t y, uint16_t g, uint32_t f)
{
	int i = heapSize++;
	heapX[i] = x;
	heapY[i] = y;
	heapG[i] = g;
	heapF[i] = f;

	while (i > 0)
	{
		int parent = (i - 1) / 2;
		if (heapF[parent] <= heapF[i])
			break;

		uint8_t tx = heapX[parent];
		heapX[parent] = heapX[i];
		heapX[i] = tx;
		uint8_t ty = heapY[parent];
		heapY[parent] = heapY[i];
		heapY[i] = ty;
		uint16_t tg = heapG[parent];
		heapG[parent] = heapG[i];
		heapG[i] = tg;
		uint32_t tf = heapF[parent];
		heapF[parent] = heapF[i];
		heapF[i] = tf;
		i = parent;
	}
}

bool popHeapToCurrent()
{
	while (heapSize > 0)
	{
		uint8_t px = heapX[0];
		uint8_t py = heapY[0];
		uint16_t pg = heapG[0];

		heapSize--;
		heapX[0] = heapX[heapSize];
		heapY[0] = heapY[heapSize];
		heapG[0] = heapG[heapSize];
		heapF[0] = heapF[heapSize];

		int i = 0;
		while (true)
		{
			int l = 2 * i + 1;
			int r = l + 1;
			int smallest = i;

			if (l < heapSize && heapF[l] < heapF[smallest])
				smallest = l;
			if (r < heapSize && heapF[r] < heapF[smallest])
				smallest = r;
			if (smallest == i)
				break;

			uint8_t tx = heapX[i];
			heapX[i] = heapX[smallest];
			heapX[smallest] = tx;
			uint8_t ty = heapY[i];
			heapY[i] = heapY[smallest];
			heapY[smallest] = ty;
			uint16_t tg = heapG[i];
			heapG[i] = heapG[smallest];
			heapG[smallest] = tg;
			uint32_t tf = heapF[i];
			heapF[i] = heapF[smallest];
			heapF[smallest] = tf;
			i = smallest;
		}

		if (isClosed(px, py))
			continue;
		if (getKnownGScore(px, py) != pg)
			continue;

		currentnode[0] = px;
		currentnode[1] = py;
		currentnode[2] = pg;
		currentnode[3] = getTargetDistance(px, py);
		currentnode[4] = field[px][py];
		return true;
	}

	return false;
}

void setClosed(uint16_t x, uint16_t y)
{
	closedStamp[x][y] = searchStamp;
}

void exploreNode()
{
	for (int i = 0; i < 8; i++)
	{
		int nx = int(currentnode[0]) + dx[i];
		int ny = int(currentnode[1]) + dy[i];

		if (nx < 0 || nx >= kFieldWidth || ny < 0 || ny >= kFieldHeight)
			continue;

		uint8_t ux = (uint8_t)nx;
		uint8_t uy = (uint8_t)ny;
		if (field[ux][uy] == 255)
			continue;
		if (isClosed(ux, uy))
			continue;

		uint16_t moveG = currentnode[2] + ((dx[i] == 0 || dy[i] == 0) ? 10 : 14);
		if (moveG < getKnownGScore(ux, uy))
		{
			setKnownGScore(ux, uy, moveG);
			origin[ux][uy] = i;
			uint32_t f = (uint32_t)getTargetDistance(ux, uy) + (uint32_t)moveG + (uint32_t)field[ux][uy];
			pushHeap(ux, uy, moveG, f);
		}
	}
}

void resetSearchState()
{
	searchStamp++;
	if (searchStamp == 0)
	{
		searchStamp = 1;
		for (uint16_t x = 0; x < kFieldWidth; x++)
			for (uint16_t y = 0; y < kFieldHeight; y++)
			{
				gStamp[x][y] = 0;
				closedStamp[x][y] = 0;
			}
	}

	heapSize = 0;
}

void buildPath(uint8_t (&path)[kPathMaxLength][2], uint16_t &pathLength)
{
	uint8_t reversedPath[kPathMaxLength][2];
	uint16_t reversedLength = 0;

	reversedPath[reversedLength][0] = currentnode[0];
	reversedPath[reversedLength][1] = currentnode[1];
	reversedLength++;

	while (!(currentnode[0] == currentStart[0] && currentnode[1] == currentStart[1]))
	{
		uint8_t originDirection = origin[currentnode[0]][currentnode[1]];
		currentnode[0] = currentnode[0] - dx[originDirection];
		currentnode[1] = currentnode[1] - dy[originDirection];

		if (reversedLength >= kPathMaxLength)
		{
			pathLength = 0;
			return;
		}

		reversedPath[reversedLength][0] = currentnode[0];
		reversedPath[reversedLength][1] = currentnode[1];
		reversedLength++;
	}

	for (uint16_t i = 0; i < reversedLength; i++)
	{
		path[i][0] = reversedPath[reversedLength - 1 - i][0];
		path[i][1] = reversedPath[reversedLength - 1 - i][1];
	}

	pathLength = reversedLength;
}
} // namespace

bool findPath(uint8_t startX, uint8_t startY,
			  uint8_t targetX, uint8_t targetY,
			  uint8_t ballX, uint8_t ballY,
			  uint8_t robotX, uint8_t robotY,
			  uint8_t (&path)[kPathMaxLength][2],
			  uint16_t &pathLength)
{
	ensureFieldCopyInitialized();
	restoreField();

	currentStart[0] = startX;
	currentStart[1] = startY;
	currentTarget[0] = targetX;
	currentTarget[1] = targetY;

	placeCenteredMask(Ball, ballX, ballY);
	placeCenteredMask(robot, robotX, robotY);

	pathFound = true;
	pathLength = 0;
	resetSearchState();

	uint8_t targetWeight = field[currentTarget[0]][currentTarget[1]];
	field[currentTarget[0]][currentTarget[1]] = 0;

	currentnode[0] = currentStart[0];
	currentnode[1] = currentStart[1];
	currentnode[2] = 0;
	currentnode[3] = getTargetDistance(currentnode[0], currentnode[1]);
	currentnode[4] = field[currentnode[0]][currentnode[1]];

	setKnownGScore(currentnode[0], currentnode[1], 0);

	while (!(currentnode[0] == currentTarget[0] && currentnode[1] == currentTarget[1]))
	{
		exploreNode();
		if (!popHeapToCurrent())
		{
			pathFound = false;
			break;
		}

		setClosed(currentnode[0], currentnode[1]);
	}

	field[currentTarget[0]][currentTarget[1]] = targetWeight;

	if (!pathFound)
		return false;

	buildPath(path, pathLength);
	return pathLength > 0;
}
} // namespace pathfinder
