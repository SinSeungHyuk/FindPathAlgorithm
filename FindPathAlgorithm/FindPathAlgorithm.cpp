// FindPathAlgorithm.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//

#include "pch.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <queue>

using namespace std;


// =============================== 다익스트라 ================================

int INF = 999999;
vector<vector<int>> Dijkstra(vector<vector<int>> Edge, int start_node) {
    // 거리,노드 번호 페어가 들어갈 우선순위 큐
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    vector<int> dist(Edge.size(), INF); // 각 정점의 거리를 큰 값으로 초기화
    vector<int> track(Edge.size(), -1); // 해당 정점에서 가장 가까운 정점

    dist[start_node] = 0; // 시작지점 거리 0
    pq.push({dist[start_node], start_node}); 

    while (pq.size()) {
        int S = pq.top().second; // 큐에서 가장 거리가 짧은 정점 번호
        for (int D = 0; D < Edge.size(); D++) { // D: 계산을 위해 방문한 노드
            if (Edge[S][D] > 0) { // 현재 노드 S에서 방문한 노드까지의 거리가 0 초과
                int path = dist[S] + Edge[S][D];
                if (path < dist[D]) { // 시작점->S + S->D 거리 vs 시작점->D 거리
                    dist[D] = path;
                    track[D] = S;
                    pq.push({ dist[D], D });
                }
            }
        }
        pq.pop();
    }

    vector<vector<int>> answer;
    answer.push_back(dist);
    answer.push_back(track);
    return answer;
}

void print_dist(vector<int> dist) { // 전체 dist 출력
    for (int i = 0; i < dist.size(); i++)
        cout << "dist[" << i << "] = " << dist[i] << '\n';
    cout << "\n\n";
}
void print_track(vector<int> prev, int start, int dest) {
    // start -> dest(도착) 까지 가는데 걸리는 루트
    int node = dest;
    vector<int> track;
    while (node != start) {
        track.push_back(node);
        node = prev[node];
    }
    cout << "시작 노드 " << start << "에서 도착 노드" << dest << "까지\n" << start << " > ";
    while (track.size()) {
        cout << track.back();
        if (track.size() > 1) cout << " > ";

        track.pop_back();
    }
}


// =============================== A* ==================================

struct Node {
    int y, x;
    int G, H;
    pair<int, int> parent;
};
struct cmp {
    bool operator() (Node p1, Node p2) {
        if (p1.G + p1.H > p2.G + p2.H) return true;
        else if (p1.G + p1.H == p2.G + p2.H) {
            if (p1.G < p2.G) return true;
            else return false;
        }
        else return false;
    }
};

void print_map(vector<vector<int>> map) { // 맵 출력 함수
    for (int i = 0; i < map.size(); i++) {

        for (int j = 0; j < map.size(); j++) cout << map[i][j] << " ";
        cout << '\n';
    }
}

void Astar(vector<vector<int>> map, pair<int,int> start, pair<int,int> goal) {
    priority_queue<Node, vector<Node>, cmp> pq;
    bool visit[100][100] = { 0, };
    vector<Node> close_list;
    vector<vector<int>> result = map;

    Node s;
    s.x = start.second;
    s.y = start.first;
    s.G = 0;
    s.H = (abs(goal.first - start.first) + abs(goal.second - start.second)) * 10;
    s.parent = { -1,-1 };

    visit[s.y][s.x] = true;
    pq.push(s);

    int dx[8] = { 0,1,0,-1,1,1,-1,-1 };
    int dy[8] = { -1,0,1,0,-1,1,1,-1 };

    while (pq.size()) {
        int x = pq.top().x;
        int y = pq.top().y;
        int G = pq.top().G;
        result[y][x] = 8;
        close_list.push_back(pq.top());
        pq.pop();
        
        if (x == goal.second && y == goal.first) break;

        Node next;
        for (int i = 0; i < 8; i++) {
            int nX = x + dx[i];
            int nY = y + dy[i];
            if (nX >= 0 && nX < map.size() && nY >= 0 && nY < map.size()) {
                if (map[nY][nX] != 1 && visit[nY][nX] == false) {
                    next.x = nX;
                    next.y = nY;
                    next.G = (i > 3) ? G + 14 : G + 10;
                    next.H = (abs(goal.first - nY) + abs(goal.second - nX)) * 10;
                    next.parent = { y,x };
                    pq.push(next);
                    visit[nY][nX] = true;
                    result[nY][nX] = 9;
                }
            }
        }
    }

    int bx = close_list.back().x;
    int by = close_list.back().y;
    while (close_list.size()) {
        if (bx == close_list.back().x && by == close_list.back().y) {
            result[by][bx] = 4;
            bx = close_list.back().parent.second;
            by = close_list.back().parent.first;
        }
        close_list.pop_back();
    }

    print_map(result);
}



int main()
{
    // 0은 자기자신, -1은 연결되지 않은 vertex
    // 1 이상은 연결된 경우의 가중치
    //vector<vector<int>> Edge = {
    //    {0, 25, 40, 30, -1, -1},//0 == A
    //    {25, 0, 5, -1, 20, -1},	//1 == B
    //    {40, 5, 0, -1, -1, 5},	//2 == C
    //    {30, -1, -1, 0, -1, 15},//3 == D
    //    {-1, 20, -1, -1, 0, 10},//4 == E
    //    {-1, -1, 5, 15, 10, 0}, //5 == F
    //};

    //vector<vector<int>> result = Dijkstra(Edge, 0);
    //print_dist(result[0]);
    //print_track(result[1], 0, 5);//A에서 F까지 가는 경로


    // ======================= A* ====================
    int size = 7;
    // 6 : 도착, 5: 출발, 0:길, 1:벽
    vector<vector<int>> map(size, vector<int>(size, 0));
    pair<int, int> start, goal;
    start = { 4, 4 };
    goal = { 8, 6 };
    map = { {0,0,0,0,0,0,0,0,0,0},
            {0,0,0,0,0,0,0,0,0,0},
            {0,0,1,1,1,1,1,1,0,0},
            {0,0,0,0,0,0,0,1,0,0},
            {0,0,0,0,5,0,0,1,0,0},
            {0,0,0,0,0,0,0,1,0,0},
            {0,0,1,1,1,1,1,1,0,0},
            {0,0,0,0,1,0,0,0,0,0},
            {0,0,0,0,1,0,6,0,0,0},
            {0,0,0,0,1,0,0,0,0,0},
    };

    Astar(map, start, goal);
    //print_map(map);
}

