#pragma GCC optimize("O3,unroll-loops")
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <numeric>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include "jsoncpp/json.h"
using namespace std;
namespace {
constexpr int kSide = 11;
constexpr int kCellCount = kSide * kSide;
constexpr int kTop = 121;
constexpr int kBottom = 122;
constexpr int kLeft = 123;
constexpr int kRight = 124;
constexpr int kDsuSize = 125;
constexpr int kMaxNodes = 220000;
constexpr int kPrepare = 256;
constexpr int kLocalDis = 3;
constexpr int kInf = 1000000000;
constexpr double kTimeLimitSeconds = 0.88;
constexpr double kExploreSelf = 0.62;
constexpr double kExploreOpp = 0.42;
constexpr double kPriorCoef = 0.18;
constexpr array<int, kCellCount> kThirdMoveBook = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,80,40,45,80,48,50,45,-1,-1,35,73,34,40,40,73,73,45,-1,-1,-1,73,80,80,50,80,61,73,80,45,45,-1,80,80,45,45,50,42,80,80,50,58,40,70,45,59,45,80,80,80,80,72,80,84,85,51,45,45,70,80,80,80,51,80,59,80,80,51,45,80,70,70,80,72,60,50,59,45,69,80,80,80,70,70,80,72,42,71,100,80,80,70,96,70,70,80,40,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,};
constexpr array<int, kCellCount> kFourthMoveBook = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,40,40,-1,40,40,40,28,40,95,51,62,71,40,40,40,71,40,96,47,83,62,95,95,36,95,95,95,50,95,83,83,40,20,40,71,71,71,71,71,71,62,29,40,40,92,95,40,71,71,71,40,40,40,96,96,40,40,59,71,81,40,40,84,40,96,40,40,93,90,-1,70,71,50,50,40,107,40,47,40,79,71,71,40,40,84,50,50,40,40,40,40,40,40,50,50,40,40,40,40,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,};
const int kDr[6] = {-1, -1, 0, 0, 1, 1};
const int kDc[6] = {0, 1, -1, 1, -1, 0};
uint8_t row_of_id[kCellCount], col_of_id[kCellCount];
uint8_t neighbors[kCellCount][6], neighbor_count[kCellCount];
uint8_t bridge_far[kCellCount][6], bridge_mid_a[kCellCount][6], bridge_mid_b[kCellCount][6], bridge_count[kCellCount];
mt19937 rng([] {
    if (const char* seed_text = getenv("HEX_SEED")) {
        char* end = nullptr;
        const unsigned long seed = strtoul(seed_text, &end, 10);
        if (end != seed_text && *end == '\0') return static_cast<unsigned>(seed);
    }
    return static_cast<unsigned>(chrono::steady_clock::now().time_since_epoch().count());
}());
chrono::steady_clock::time_point search_started_at;
inline bool inside(int row, int col) {
    return static_cast<unsigned>(row) < kSide && static_cast<unsigned>(col) < kSide;
}
inline int cell_id(int row, int col) { return row * kSide + col; }
inline int row_of(int id) { return row_of_id[id]; }
inline int col_of(int id) { return col_of_id[id]; }
inline bool time_left() {
    return chrono::duration<double>(chrono::steady_clock::now() - search_started_at).count() < kTimeLimitSeconds;
}
void init_geometry() {
    memset(neighbor_count, 0, sizeof(neighbor_count));
    memset(bridge_count, 0, sizeof(bridge_count));
    for (int id = 0; id < kCellCount; ++id) {
        const int row = id / kSide, col = id % kSide;
        row_of_id[id] = static_cast<uint8_t>(row);
        col_of_id[id] = static_cast<uint8_t>(col);
        for (int dir = 0; dir < 6; ++dir) {
            const int nr = row + kDr[dir], nc = col + kDc[dir];
            if (inside(nr, nc)) neighbors[id][neighbor_count[id]++] = static_cast<uint8_t>(cell_id(nr, nc));
        }
        auto add_bridge = [&](int far_r, int far_c, int mid1_r, int mid1_c, int mid2_r, int mid2_c) {
            if (!inside(far_r, far_c) || !inside(mid1_r, mid1_c) || !inside(mid2_r, mid2_c)) return;
            const int idx = bridge_count[id]++;
            bridge_far[id][idx] = static_cast<uint8_t>(cell_id(far_r, far_c));
            bridge_mid_a[id][idx] = static_cast<uint8_t>(cell_id(mid1_r, mid1_c));
            bridge_mid_b[id][idx] = static_cast<uint8_t>(cell_id(mid2_r, mid2_c));
        };
        add_bridge(row - 2, col + 1, row - 1, col, row - 1, col + 1);
        add_bridge(row - 1, col - 1, row - 1, col, row, col - 1);
        add_bridge(row - 1, col + 2, row - 1, col + 1, row, col + 1);
        add_bridge(row + 1, col - 2, row + 1, col - 1, row, col - 1);
        add_bridge(row + 1, col + 1, row, col + 1, row + 1, col);
        add_bridge(row + 2, col - 1, row + 1, col - 1, row + 1, col);
    }
}
struct Board {
    array<int8_t, kCellCount> stone{};
    array<int, kDsuSize> parent{};
    int turn = 1;
    Board() { reset(); }
    void reset() {
        stone.fill(0);
        iota(parent.begin(), parent.end(), 0);
        turn = 1;
    }
    int root(int value) {
        int result = value;
        while (parent[result] != result) result = parent[result];
        while (parent[value] != result) {
            const int next = parent[value];
            parent[value] = result;
            value = next;
        }
        return result;
    }
    void unite(int lhs, int rhs) {
        lhs = root(lhs);
        rhs = root(rhs);
        if (lhs != rhs) parent[lhs] = rhs;
    }
    bool empty(int id) const { return stone[id] == 0; }
    bool red_won() { return root(kTop) == root(kBottom); }
    bool blue_won() { return root(kLeft) == root(kRight); }
    bool play_id(int id, int color) {
        assert(0 <= id && id < kCellCount && stone[id] == 0);
        stone[id] = static_cast<int8_t>(color);
        for (int i = 0; i < neighbor_count[id]; ++i) {
            const int nid = neighbors[id][i];
            if (stone[nid] == color) unite(id, nid);
        }
        const int row = row_of(id), col = col_of(id);
        if (color == 1) {
            if (row == 0) unite(id, kTop);
            if (row == kSide - 1) unite(id, kBottom);
        } else {
            if (col == 0) unite(id, kLeft);
            if (col == kSide - 1) unite(id, kRight);
        }
        turn = -color;
        return color == 1 ? red_won() : blue_won();
    }
    bool play_current(int row, int col) { return play_id(cell_id(row, col), turn); }
};
vector<int> legal_moves(const Board& board) {
    vector<int> moves;
    moves.reserve(kCellCount);
    for (int id = 0; id < kCellCount; ++id) {
        if (board.empty(id)) moves.push_back(id);
    }
    return moves;
}
int stone_count(const Board& board) {
    int count = 0;
    for (int id = 0; id < kCellCount; ++id) count += board.stone[id] != 0;
    return count;
}
int opening_book_move(const Board& board) {
    const int occupied = stone_count(board);
    auto legal_book_move = [&](int id) { return 0 <= id && id < kCellCount && board.empty(id) ? id : -1; };
    if (occupied == 1) return legal_book_move(cell_id(7, 3));
    const int swap_anchor = cell_id(1, 2), reply_anchor = cell_id(7, 3);
    if ((occupied == 2 && board.stone[swap_anchor] != 0) ||
        (occupied == 3 && board.stone[swap_anchor] != 0 && board.stone[reply_anchor] != 0)) {
        int pattern_id = -1;
        for (int id = 0; id < kCellCount; ++id) {
            if (board.stone[id] == 0 || id == swap_anchor || (occupied == 3 && id == reply_anchor)) continue;
            pattern_id = id;
        }
        if (pattern_id != -1) {
            return legal_book_move(occupied == 2 ? kThirdMoveBook[pattern_id] : kFourthMoveBook[pattern_id]);
        }
    }
    return -1;
}
int immediate_win(const Board& board, const vector<int>& moves, int color) {
    for (int id : moves) {
        Board trial = board;
        if (trial.play_id(id, color)) return id;
    }
    return -1;
}
struct PathInfo {
    array<int, kCellCount> a{}, b{};
    int best = kInf;
};
void path_dist(const Board& board, int color, bool rev, array<int, kCellCount>& dist) {
    deque<int> q;
    dist.fill(kInf);
    for (int id = 0; id < kCellCount; ++id) {
        if (board.stone[id] == -color) continue;
        const bool edge = color == 1 ? (rev ? row_of(id) == kSide - 1 : row_of(id) == 0)
                                     : (rev ? col_of(id) == kSide - 1 : col_of(id) == 0);
        if (!edge) continue;
        dist[id] = board.stone[id] == color ? 0 : 1;
        (dist[id] ? q.push_back(id) : q.push_front(id));
    }
    while (!q.empty()) {
        const int id = q.front(); q.pop_front();
        for (int i = 0; i < neighbor_count[id]; ++i) {
            const int v = neighbors[id][i];
            if (board.stone[v] == -color) continue;
            const int w = board.stone[v] == color ? 0 : 1;
            if (dist[id] + w < dist[v]) {
                dist[v] = dist[id] + w;
                (w ? q.push_back(v) : q.push_front(v));
            }
        }
    }
}
PathInfo paths(const Board& board, int color) {
    PathInfo p;
    path_dist(board, color, false, p.a);
    path_dist(board, color, true, p.b);
    for (int id = 0; id < kCellCount; ++id) {
        if (board.stone[id] == -color || p.a[id] >= kInf || p.b[id] >= kInf) continue;
        p.best = min(p.best, p.a[id] + p.b[id] - (board.stone[id] == color ? 0 : 1));
    }
    return p;
}
int span_after(const PathInfo& p, int id) {
    return p.a[id] >= kInf || p.b[id] >= kInf ? kInf : p.a[id] + p.b[id] - 1;
}
double shape_score(const Board& board, int id, int color) {
    const int row = row_of(id), col = col_of(id);
    double score = 10.0 - abs(row - 5) - abs(col - 5);
    score += color == 1 ? (5.0 - abs(row - 5)) * 4.0 : (5.0 - abs(col - 5)) * 4.0;
    for (int i = 0; i < neighbor_count[id]; ++i) {
        const int s = board.stone[neighbors[id][i]];
        if (s == color) score += 24.0;
        if (s == -color) score += 14.0;
    }
    for (int i = 0; i < bridge_count[id]; ++i) {
        const int far = bridge_far[id][i], a = bridge_mid_a[id][i], b = bridge_mid_b[id][i];
        if (board.stone[far] == color && board.empty(a) && board.empty(b)) score += 40.0;
        if (board.stone[far] == -color && board.empty(a) && board.empty(b)) score += 28.0;
    }
    return score;
}
array<bool, kCellCount> root_ban_mask(const Board& board) {
    array<bool, kCellCount> banned{};
    array<int, kCellCount> dis;
    queue<int> q;
    dis.fill(-1);
    for (int id = 0; id < kCellCount; ++id) {
        if (board.stone[id] != 0) {
            dis[id] = 0;
            q.push(id);
        }
    }
    if (q.empty()) return banned;
    while (!q.empty()) {
        const int id = q.front();
        q.pop();
        if (dis[id] == kLocalDis) continue;
        for (int i = 0; i < neighbor_count[id]; ++i) {
            const int nid = neighbors[id][i];
            if (dis[nid] == -1) {
                dis[nid] = dis[id] + 1;
                q.push(nid);
            }
        }
    }
    for (int id = 0; id < kCellCount; ++id) banned[id] = board.empty(id) && dis[id] > kLocalDis;
    return banned;
}
struct RandomVisit {
    array<int, kCellCount> seq{};
    array<int, kCellCount> pos{};
    int n = 0;
    void init(const vector<int>& moves) {
        n = static_cast<int>(moves.size());
        for (int i = 0; i < n; ++i) seq[i] = moves[i];
        shuffle(seq.begin(), seq.begin() + n, rng);
        for (int i = 0; i < n; ++i) pos[seq[i]] = i;
    }
    void erase(int move) {
        const int at = pos[move];
        const int last = seq[n - 1];
        seq[at] = last;
        pos[last] = at;
        --n;
    }
    vector<int> remaining() const {
        return vector<int>(seq.begin(), seq.begin() + n);
    }
};
struct Node {
    array<int, kCellCount> child{};
    int visits = 0;
    int wins = 0;
    int sons = 0;
};
vector<Node> tree;
array<double, kCellCount> prior{};
array<bool, kCellCount> banned_root{};
int root_color = 1;
int new_node() {
    tree.emplace_back();
    return static_cast<int>(tree.size()) - 1;
}
void build_prior(const Board& root, const vector<int>& root_moves) {
    array<int, kCellCount> warm{};
    vector<int> moves = root_moves;
    const int prepare = min(kPrepare, max(48, 3 * static_cast<int>(moves.size())));
    for (int t = 0; t < prepare; ++t) {
        shuffle(moves.begin(), moves.end(), rng);
        Board board = root;
        int color = root.turn;
        vector<pair<int, int>> played;
        for (int id : moves) {
            played.push_back({id, color});
            if (board.play_id(id, color)) {
                if (color == root.turn) {
                    for (const auto& item : played) {
                        if (item.second == root.turn) ++warm[item.first];
                    }
                }
                break;
            }
            color = -color;
        }
    }
    prior.fill(0.0);
    for (int id : root_moves) {
        const double warm_score = static_cast<double>(warm[id]) / max(1, prepare);
        const double static_score = max(0.0, shape_score(root, id, root.turn)) / 180.0;
        prior[id] = kPriorCoef * warm_score + 0.04 * static_score;
    }
}
int tactical_move(const Board& board, const RandomVisit& can, int color) {
    const vector<int> moves = can.remaining();
    int id = immediate_win(board, moves, color);
    if (id != -1) return id;
    id = immediate_win(board, moves, -color);
    return id;
}
int bridge_reply_move(const Board& board, int color) {
    for (int id = 0; id < kCellCount; ++id) {
        if (board.stone[id] != color) continue;
        for (int i = 0; i < bridge_count[id]; ++i) {
            const int far = bridge_far[id][i], a = bridge_mid_a[id][i], b = bridge_mid_b[id][i];
            if (board.stone[far] != color) continue;
            if (board.stone[a] == -color && board.empty(b)) return b;
            if (board.stone[b] == -color && board.empty(a)) return a;
        }
    }
    return -1;
}
int rollout_move(const Board& board, const RandomVisit& can, int color, int last_move) {
    const vector<int> moves = can.remaining();
    int forced = immediate_win(board, moves, color);
    if (forced != -1) return forced;
    forced = immediate_win(board, moves, -color);
    if (forced != -1) return forced;
    forced = bridge_reply_move(board, color);
    if (forced != -1 && board.empty(forced)) return forced;
    const PathInfo own = paths(board, color), opp = paths(board, -color);
    int best = moves[rng() % moves.size()];
    double best_score = -1e100;
    for (int id : moves) {
        bool local = last_move == -1;
        if (last_move != -1) {
            const int dr = row_of(id) - row_of(last_move), dc = col_of(id) - col_of(last_move);
            local = max({abs(dr), abs(dc), abs(dr + dc)}) <= 2;
        }
        double score = (local ? 25.0 : 0.0) + shape_score(board, id, color) + 0.70 * shape_score(board, id, -color);
        const int os = span_after(own, id), ps = span_after(opp, id);
        if (os < kInf) score += 90.0 / (1 + os);
        if (ps < kInf) score += 75.0 / (1 + ps);
        if (os == own.best) score += 65.0;
        if (ps == opp.best) score += 50.0;
        if ((rng() & 31) == 0) score += static_cast<int>(rng() & 15);
        if (score > best_score) best_score = score, best = id;
    }
    return best;
}
double ucb_value(int parent, int son, int move, int depth) {
    const double rate = static_cast<double>(tree[son].wins) / tree[son].visits;
    const double exploit = (depth & 1) ? 1.0 - rate : rate;
    const double explore = ((depth & 1) ? kExploreOpp : kExploreSelf) * sqrt(log(tree[parent].visits + 1.0) / tree[son].visits);
    return exploit + explore + prior[move];
}
int pick_tree_move(int node, const RandomVisit& can, int depth, bool playout) {
    if (playout) return can.seq[rng() % can.n];
    if (tree[node].sons < can.n) {
        for (int i = 0; i < can.n; ++i) {
            const int move = can.seq[i];
            if (!tree[node].child[move] && (depth != 0 || !banned_root[move])) return move;
        }
        for (int i = 0; i < can.n; ++i) {
            const int move = can.seq[i];
            if (!tree[node].child[move]) return move;
        }
    }
    int best = can.seq[0];
    double best_value = -1e100;
    for (int i = 0; i < can.n; ++i) {
        const int move = can.seq[i];
        if (depth == 0 && banned_root[move]) continue;
        const int son = tree[node].child[move];
        if (!son) continue;
        const double value = ucb_value(node, son, move, depth);
        if (value > best_value) {
            best_value = value;
            best = move;
        }
    }
    return best_value > -1e90 ? best : can.seq[rng() % can.n];
}
pair<int, int> search(Board root) {
    vector<int> root_moves = legal_moves(root);
    int move = immediate_win(root, root_moves, root.turn);
    if (move != -1) return {row_of(move), col_of(move)};
    move = immediate_win(root, root_moves, -root.turn);
    if (move != -1) return {row_of(move), col_of(move)};
    move = bridge_reply_move(root, root.turn);
    if (move != -1) return {row_of(move), col_of(move)};
    root_color = root.turn;
    banned_root = root_ban_mask(root);
    build_prior(root, root_moves);
    tree.clear();
    tree.reserve(kMaxNodes);
    new_node();
    array<int, kCellCount + 1> path{};
    int iterations = 0;
    while (time_left() && static_cast<int>(tree.size()) < kMaxNodes - 2) {
        Board board = root;
        RandomVisit can;
        can.init(root_moves);
        int node = 0, color = root_color, depth = 0, top = 0, winner = 0, last_move = -1;
        bool playout = false;
        path[top++] = node;
        while (can.n > 0) {
            int chosen = playout ? rollout_move(board, can, color, last_move) : tactical_move(board, can, color);
            if (chosen == -1) chosen = pick_tree_move(node, can, depth, false);
            if (!playout && !tree[node].child[chosen]) {
                tree[node].child[chosen] = new_node();
                ++tree[node].sons;
                playout = true;
            }
            if (!playout || tree[node].child[chosen]) {
                node = tree[node].child[chosen];
                path[top++] = node;
            }
            can.erase(chosen);
            if (board.play_id(chosen, color)) {
                winner = color;
                break;
            }
            last_move = chosen;
            color = -color;
            ++depth;
        }
        if (winner == 0) winner = board.red_won() ? 1 : -1;
        for (int i = 0; i < top; ++i) {
            ++tree[path[i]].visits;
            if (winner == root_color) ++tree[path[i]].wins;
        }
        if ((++iterations & 255) == 0 && !time_left()) break;
    }
    int answer = root_moves.front();
    double best_rate = -1.0;
    bool found_unbanned = false;
    for (int id : root_moves) {
        const int child = tree[0].child[id];
        if (!child || tree[child].visits <= 1) continue;
        const double rate = static_cast<double>(tree[child].wins) / tree[child].visits;
        const bool usable = !banned_root[id];
        if ((usable && !found_unbanned) || (usable == found_unbanned && rate > best_rate)) {
            answer = id;
            best_rate = rate;
            found_unbanned = usable;
        }
    }
    return {row_of(answer), col_of(answer)};
}
Json::Value make_response(int row, int col) {
    Json::Value response;
    response["x"] = row;
    response["y"] = col;
    return response;
}
}  // namespace
int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    init_geometry();
    search_started_at = chrono::steady_clock::now();
    string payload;
    getline(cin, payload);
    if (payload.empty()) return 0;
    Json::CharReaderBuilder reader_builder;
    Json::Value input_json;
    string errors;
    unique_ptr<Json::CharReader> reader(reader_builder.newCharReader());
    if (!reader->parse(payload.data(), payload.data() + payload.size(), &input_json, &errors)) return 0;
    Board board;
    const int finished_turns = input_json["responses"].size();
    for (int turn = 0; turn < finished_turns; ++turn) {
        const int request_x = input_json["requests"][turn]["x"].asInt();
        const int request_y = input_json["requests"][turn]["y"].asInt();
        if (request_x >= 0 && request_y >= 0) board.play_current(request_x, request_y);
        const int response_x = input_json["responses"][turn]["x"].asInt();
        const int response_y = input_json["responses"][turn]["y"].asInt();
        if (response_x >= 0 && response_y >= 0) board.play_current(response_x, response_y);
    }
    const int current_x = input_json["requests"][finished_turns]["x"].asInt();
    const int current_y = input_json["requests"][finished_turns]["y"].asInt();
    bool forced_start = false;
    if (current_x >= 0 && current_y >= 0) {
        board.play_current(current_x, current_y);
    } else {
        forced_start = input_json["requests"][0].isMember("forced_x");
    }
    pair<int, int> decision;
    if (forced_start) {
        decision = {1, 2};
    } else if (const int book_move = opening_book_move(board); book_move != -1) {
        decision = {row_of(book_move), col_of(book_move)};
    } else {
        decision = search(board);
    }
    Json::Value result;
    result["response"] = make_response(decision.first, decision.second);
    Json::StreamWriterBuilder writer_builder;
    writer_builder["indentation"] = "";
    cout << Json::writeString(writer_builder, result) << '\n';
    return 0;
}
