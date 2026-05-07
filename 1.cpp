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
#include <limits>
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
constexpr double kTimeLimitSeconds = 0.88;
constexpr double kExplore = 0.55;
constexpr int kRolloutRandomPercent = 7;
constexpr int kNodeReserve = 180000;
constexpr int kInf = 1000000000;

const int kDr[6] = {-1, -1, 0, 0, 1, 1};
const int kDc[6] = {0, 1, -1, 1, -1, 0};

uint8_t row_of_id[kCellCount];
uint8_t col_of_id[kCellCount];
uint8_t neighbors[kCellCount][6];
uint8_t neighbor_count[kCellCount];
uint8_t near_cells[kCellCount][25];
uint8_t near_count[kCellCount];
uint8_t bridge_far[kCellCount][6];
uint8_t bridge_mid_a[kCellCount][6];
uint8_t bridge_mid_b[kCellCount][6];
uint8_t bridge_count[kCellCount];

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
    memset(near_count, 0, sizeof(near_count));
    memset(bridge_count, 0, sizeof(bridge_count));

    for (int id = 0; id < kCellCount; ++id) {
        const int row = id / kSide;
        const int col = id % kSide;
        row_of_id[id] = static_cast<uint8_t>(row);
        col_of_id[id] = static_cast<uint8_t>(col);

        for (int dir = 0; dir < 6; ++dir) {
            const int nr = row + kDr[dir];
            const int nc = col + kDc[dir];
            if (inside(nr, nc)) neighbors[id][neighbor_count[id]++] = static_cast<uint8_t>(cell_id(nr, nc));
        }

        for (int dr = -2; dr <= 2; ++dr) {
            for (int dc = -2; dc <= 2; ++dc) {
                const int nr = row + dr;
                const int nc = col + dc;
                const int hex_dist = max({abs(dr), abs(dc), abs(dr + dc)});
                if (inside(nr, nc) && hex_dist <= 2) near_cells[id][near_count[id]++] = static_cast<uint8_t>(cell_id(nr, nc));
            }
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
    bool finished() { return red_won() || blue_won(); }

    bool play_id(int id, int color) {
        assert(0 <= id && id < kCellCount);
        assert(stone[id] == 0);
        stone[id] = static_cast<int8_t>(color);
        const int row = row_of(id);
        const int col = col_of(id);

        for (int i = 0; i < neighbor_count[id]; ++i) {
            const int nid = neighbors[id][i];
            if (stone[nid] == color) unite(id, nid);
        }

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

    bool play_current(int row, int col) {
        return play_id(cell_id(row, col), turn);
    }
};

struct PathInfo {
    array<int, kCellCount> from_start{};
    array<int, kCellCount> from_goal{};
    int best = kInf;
};

void path_distances(const Board& board, int color, bool reverse, array<int, kCellCount>& dist) {
    deque<int> dq;
    dist.fill(kInf);

    for (int id = 0; id < kCellCount; ++id) {
        if (board.stone[id] == -color) continue;
        const int row = row_of(id);
        const int col = col_of(id);
        const bool on_edge = color == 1 ? (reverse ? row == kSide - 1 : row == 0)
                                      : (reverse ? col == kSide - 1 : col == 0);
        if (!on_edge) continue;
        dist[id] = board.stone[id] == color ? 0 : 1;
        if (dist[id] == 0) dq.push_front(id);
        else dq.push_back(id);
    }

    while (!dq.empty()) {
        const int id = dq.front();
        dq.pop_front();
        for (int i = 0; i < neighbor_count[id]; ++i) {
            const int nid = neighbors[id][i];
            if (board.stone[nid] == -color) continue;
            const int cost = board.stone[nid] == color ? 0 : 1;
            if (dist[id] + cost < dist[nid]) {
                dist[nid] = dist[id] + cost;
                if (cost == 0) dq.push_front(nid);
                else dq.push_back(nid);
            }
        }
    }
}

PathInfo evaluate_paths(const Board& board, int color) {
    PathInfo info;
    path_distances(board, color, false, info.from_start);
    path_distances(board, color, true, info.from_goal);
    for (int id = 0; id < kCellCount; ++id) {
        if (board.stone[id] == -color) continue;
        const int own_cost = board.stone[id] == color ? 0 : 1;
        if (info.from_start[id] >= kInf || info.from_goal[id] >= kInf) continue;
        info.best = min(info.best, info.from_start[id] + info.from_goal[id] - own_cost);
    }
    return info;
}

int span_after_move(const PathInfo& info, int id) {
    if (info.from_start[id] >= kInf || info.from_goal[id] >= kInf) return kInf;
    return info.from_start[id] + info.from_goal[id] - 1;
}

double static_move_score(const Board& board, int id, int color) {
    const int row = row_of(id);
    const int col = col_of(id);
    double score = 12.0 - abs(row - kSide / 2) - abs(col - kSide / 2);

    for (int i = 0; i < neighbor_count[id]; ++i) {
        const int occupant = board.stone[neighbors[id][i]];
        if (occupant == color) score += 30.0;
        else if (occupant == -color) score += 18.0;
    }

    for (int i = 0; i < bridge_count[id]; ++i) {
        const int far = bridge_far[id][i];
        const int mid_a = bridge_mid_a[id][i];
        const int mid_b = bridge_mid_b[id][i];
        if (board.stone[far] == color && board.empty(mid_a) && board.empty(mid_b)) score += 58.0;
        if (board.stone[far] == -color && board.empty(mid_a) && board.empty(mid_b)) score += 36.0;
    }

    score += color == 1 ? (5.0 - abs(row - 5)) * 6.0 : (5.0 - abs(col - 5)) * 6.0;
    return score;
}

int immediate_win(const Board& board, const vector<int>& empties, int color) {
    for (int id : empties) {
        Board trial = board;
        if (trial.play_id(id, color)) return id;
    }
    return -1;
}

vector<int> legal_moves(const Board& board) {
    vector<int> result;
    result.reserve(kCellCount);
    for (int id = 0; id < kCellCount; ++id) {
        if (board.empty(id)) result.push_back(id);
    }
    return result;
}

int best_static_move(const Board& board) {
    vector<int> empties = legal_moves(board);
    if (empties.empty()) return -1;

    const int win = immediate_win(board, empties, board.turn);
    if (win != -1) return win;
    const int block = immediate_win(board, empties, -board.turn);
    if (block != -1) return block;

    const PathInfo own_path = evaluate_paths(board, board.turn);
    const PathInfo opp_path = evaluate_paths(board, -board.turn);
    int best = empties.front();
    double best_score = -1e100;

    for (int id : empties) {
        Board child = board;
        child.play_id(id, board.turn);
        vector<int> child_empties = legal_moves(child);

        double score = static_move_score(board, id, board.turn) + 0.85 * static_move_score(board, id, -board.turn);
        const int own_span = span_after_move(own_path, id);
        const int opp_span = span_after_move(opp_path, id);
        if (own_span < kInf) score += 130.0 / (1.0 + own_span);
        if (opp_span < kInf) score += 120.0 / (1.0 + opp_span);
        if (own_span == own_path.best) score += 95.0;
        if (opp_span == opp_path.best) score += 90.0;

        const PathInfo child_own = evaluate_paths(child, board.turn);
        const PathInfo child_opp = evaluate_paths(child, -board.turn);
        score += 160.0 * (child_opp.best - child_own.best);

        if (immediate_win(child, child_empties, -board.turn) != -1) score -= 100000.0;
        if (immediate_win(child, child_empties, board.turn) != -1) score += 650.0;

        if (score > best_score) {
            best_score = score;
            best = id;
        }
    }
    return best;
}

int choose_rollout_move(const Board& board, const vector<int>& empties, int color, int last_move) {
    const int win = immediate_win(board, empties, color);
    if (win != -1) return win;
    const int block = immediate_win(board, empties, -color);
    if (block != -1) return block;

    vector<int> candidates;
    array<bool, kCellCount> used{};
    auto push = [&](int id) {
        if (id >= 0 && id < kCellCount && board.empty(id) && !used[id]) {
            used[id] = true;
            candidates.push_back(id);
        }
    };

    if (last_move != -1) {
        for (int i = 0; i < near_count[last_move]; ++i) push(near_cells[last_move][i]);
    }
    for (int id : empties) {
        for (int i = 0; i < neighbor_count[id]; ++i) {
            if (board.stone[neighbors[id][i]] != 0) {
                push(id);
                break;
            }
        }
    }
    if (candidates.empty()) candidates = empties;

    if (empties.size() > 18 && static_cast<int>(rng() % 100) < kRolloutRandomPercent) {
        return candidates[rng() % candidates.size()];
    }

    const PathInfo own_path = evaluate_paths(board, color);
    const PathInfo opp_path = evaluate_paths(board, -color);
    int best = candidates[0];
    double best_score = -1e100;
    for (int id : candidates) {
        double score = static_move_score(board, id, color) + 0.60 * static_move_score(board, id, -color);
        const int own_span = span_after_move(own_path, id);
        const int opp_span = span_after_move(opp_path, id);
        if (own_span < kInf) score += 80.0 / (1.0 + own_span);
        if (opp_span < kInf) score += 64.0 / (1.0 + opp_span);
        if (own_span == own_path.best) score += 55.0;
        if (opp_span == opp_path.best) score += 45.0;
        if (score > best_score) {
            best_score = score;
            best = id;
        }
    }
    return best;
}

int rollout(Board board, int color) {
    vector<int> empties = legal_moves(board);
    int last_move = -1;

    while (!empties.empty()) {
        const int id = choose_rollout_move(board, empties, color, last_move);
        empties.erase(find(empties.begin(), empties.end(), id));
        if (board.play_id(id, color)) return color;
        last_move = id;

        const int placed_color = color;
        color = -color;

        for (int i = 0; i < bridge_count[id]; ++i) {
            const int far = bridge_far[id][i];
            const int mid_a = bridge_mid_a[id][i];
            const int mid_b = bridge_mid_b[id][i];
            if (board.stone[far] == placed_color && board.empty(mid_a) && board.empty(mid_b)) {
                int first = mid_a;
                int second = mid_b;
                if (rng() & 1) swap(first, second);
                auto it = find(empties.begin(), empties.end(), first);
                if (it != empties.end()) empties.erase(it);
                if (board.play_id(first, color)) return color;
                color = -color;
                it = find(empties.begin(), empties.end(), second);
                if (it != empties.end()) empties.erase(it);
                if (board.play_id(second, color)) return color;
                color = -color;
                break;
            }
        }
    }
    return board.red_won() ? 1 : -1;
}

struct Node {
    int move = -1;
    int player = 0;
    int parent = -1;
    vector<int> children;
    vector<int> untried;
    int visits = 0;
    double wins = 0.0;
    double prior = 0.0;
};

vector<Node> tree;
int root_color = 1;

vector<int> candidate_moves(const Board& board, int color) {
    vector<int> empties = legal_moves(board);
    const int win = immediate_win(board, empties, color);
    if (win != -1) return {win};
    const int block = immediate_win(board, empties, -color);
    if (block != -1) return {block};

    array<bool, kCellCount> used{};
    vector<pair<double, int>> scored;
    PathInfo own_path = evaluate_paths(board, color);
    PathInfo opp_path = evaluate_paths(board, -color);

    bool has_stone = false;
    for (int id = 0; id < kCellCount; ++id) has_stone = has_stone || board.stone[id] != 0;

    for (int id : empties) {
        bool nearby = !has_stone;
        for (int i = 0; i < near_count[id] && !nearby; ++i) nearby = board.stone[near_cells[id][i]] != 0;
        const int own_span = span_after_move(own_path, id);
        const int opp_span = span_after_move(opp_path, id);
        const bool corridor = own_span <= own_path.best + 1 || opp_span <= opp_path.best + 1;
        if (!nearby && !corridor && empties.size() > 24) continue;

        double score = static_move_score(board, id, color) + 0.72 * static_move_score(board, id, -color);
        if (own_span < kInf) score += 100.0 / (1.0 + own_span);
        if (opp_span < kInf) score += 85.0 / (1.0 + opp_span);
        if (own_span == own_path.best) score += 80.0;
        if (opp_span == opp_path.best) score += 64.0;
        scored.push_back({score, id});
        used[id] = true;
    }

    if (scored.empty()) {
        for (int id : empties) scored.push_back({static_move_score(board, id, color), id});
    }

    sort(scored.begin(), scored.end(), greater<pair<double, int>>());
    const int keep = empties.size() > 80 ? 14 : empties.size() > 55 ? 22 : empties.size() > 30 ? 34 : kCellCount;
    vector<int> result;
    for (int i = 0; i < static_cast<int>(scored.size()) && i < keep; ++i) result.push_back(scored[i].second);
    shuffle(result.begin(), result.end(), rng);
    return result;
}

int make_node(int move, int player, int parent, const Board& board, int next_color) {
    Node node;
    node.move = move;
    node.player = player;
    node.parent = parent;
    if (move != -1) node.prior = min(1.0, max(0.0, (static_move_score(board, move, player) + 50.0) / 260.0));
    node.untried = candidate_moves(board, next_color);
    tree.push_back(std::move(node));
    return static_cast<int>(tree.size()) - 1;
}

int select_child(int node_id) {
    const Node& node = tree[node_id];
    int best = node.children.front();
    double best_score = -1e100;
    const double parent_log = log(node.visits + 1.0);
    for (int child_id : node.children) {
        const Node& child = tree[child_id];
        const double visits = child.visits + 1e-9;
        const double value = child.wins / visits;
        const double ucb = value + kExplore * sqrt(parent_log / visits) + 1.8 * child.prior / (child.visits + 1.0);
        if (ucb > best_score) {
            best_score = ucb;
            best = child_id;
        }
    }
    return best;
}

pair<int, int> search(Board root_board) {
    vector<int> root_legal = legal_moves(root_board);
    int move = immediate_win(root_board, root_legal, root_board.turn);
    if (move != -1) return {row_of(move), col_of(move)};
    move = immediate_win(root_board, root_legal, -root_board.turn);
    if (move != -1) return {row_of(move), col_of(move)};
    const int static_fallback = best_static_move(root_board);

    root_color = root_board.turn;
    tree.clear();
    tree.reserve(kNodeReserve);
    make_node(-1, -root_color, -1, root_board, root_color);

    int iterations = 0;
    while (time_left() && static_cast<int>(tree.size()) < kNodeReserve - 2) {
        Board board = root_board;
        int node_id = 0;
        int color = root_color;

        while (tree[node_id].untried.empty() && !tree[node_id].children.empty()) {
            node_id = select_child(node_id);
            if (board.play_id(tree[node_id].move, tree[node_id].player)) break;
            color = -tree[node_id].player;
        }

        int winner = 0;
        if (tree[node_id].move != -1) {
            if (tree[node_id].player == 1 && board.red_won()) winner = 1;
            if (tree[node_id].player == -1 && board.blue_won()) winner = -1;
        }

        if (winner == 0 && !tree[node_id].untried.empty()) {
            int idx = rng() % tree[node_id].untried.size();
            if (node_id == 0 && static_fallback != -1) {
                for (int i = 0; i < static_cast<int>(tree[node_id].untried.size()); ++i) {
                    if (tree[node_id].untried[i] == static_fallback) {
                        idx = i;
                        break;
                    }
                }
            }
            const int next_move = tree[node_id].untried[idx];
            tree[node_id].untried[idx] = tree[node_id].untried.back();
            tree[node_id].untried.pop_back();
            winner = board.play_id(next_move, color) ? color : 0;
            const int child_id = make_node(next_move, color, node_id, board, -color);
            tree[node_id].children.push_back(child_id);
            node_id = child_id;
            color = -color;
        }

        if (winner == 0) winner = rollout(board, color);

        for (int cur = node_id; cur != -1; cur = tree[cur].parent) {
            ++tree[cur].visits;
            if (winner == root_color) tree[cur].wins += 1.0;
            else tree[cur].wins -= 0.5;
        }

        if ((++iterations & 63) == 0 && !time_left()) break;
    }

    int best_child = -1;
    int best_visits = -1;
    double best_rate = -1e100;
    for (int child_id : tree[0].children) {
        const Node& child = tree[child_id];
        const double rate = child.visits > 0 ? child.wins / child.visits : -1e100;
        if (child.visits > best_visits || (child.visits == best_visits && rate > best_rate)) {
            best_visits = child.visits;
            best_rate = rate;
            best_child = child_id;
        }
    }

    if (best_child != -1) return {row_of(tree[best_child].move), col_of(tree[best_child].move)};
    if (static_fallback != -1) return {row_of(static_fallback), col_of(static_fallback)};
    if (!root_legal.empty()) return {row_of(root_legal.front()), col_of(root_legal.front())};
    return {0, 0};
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
    if (!reader->parse(payload.data(), payload.data() + payload.size(), &input_json, &errors)) {
        return 0;
    }

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

    pair<int, int> decision = forced_start ? make_pair(1, 2) : search(board);

    Json::Value result;
    result["response"] = make_response(decision.first, decision.second);

    Json::StreamWriterBuilder writer_builder;
    writer_builder["indentation"] = "";
    cout << Json::writeString(writer_builder, result) << '\n';
    return 0;
}
