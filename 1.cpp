#pragma GCC optimize("O3,unroll-loops")
#include <bits/stdc++.h>
#include "jsoncpp/json.h"

using namespace std;
namespace {

constexpr int kSide = 11;
constexpr int kCells = kSide * kSide;
constexpr int kTop = 121;
constexpr int kBottom = 122;
constexpr int kLeft = 123;
constexpr int kRight = 124;
constexpr int kMaxDsu = 125;
constexpr int kNodeLimit = 10000005;
constexpr double kThinkSeconds = 0.90;
constexpr double kExplore = 0.40;

mt19937 rng(1);

inline int to_id(int row, int col) { return row * kSide + col; }
inline int row_of(int id) { return id / kSide; }
inline int col_of(int id) { return id % kSide; }
inline bool inside(int row, int col) {
    return 0 <= row && row < kSide && 0 <= col && col < kSide;
}

const int kDr[6] = {-1, -1, 0, 0, 1, 1};
const int kDc[6] = {0, 1, -1, 1, -1, 0};

vector<pair<int, int>> cells;
int generated_nodes = 1;
int search_radius = 2;
clock_t started_at;

class Position {
public:
    array<array<int8_t, kSide>, kSide> stone{};
    array<int, kMaxDsu> parent{};
    int turn = 1;

    Position() { reset(); }

    void reset() {
        for (auto& row : stone) row.fill(0);
        iota(parent.begin(), parent.end(), 0);
        turn = 1;
    }

    int root(int x) {
        if (parent[x] == x) return x;
        return parent[x] = root(parent[x]);
    }

    int color(int row, int col) const { return stone[row][col]; }

    void link(int a, int b) { parent[root(a)] = root(b); }

    void play(int row, int col) {
        assert(inside(row, col));
        assert(stone[row][col] == 0);

        stone[row][col] = static_cast<int8_t>(turn);
        const int here = to_id(row, col);
        for (int dir = 0; dir < 6; ++dir) {
            const int nr = row + kDr[dir];
            const int nc = col + kDc[dir];
            if (inside(nr, nc)) {
                if (stone[nr][nc] == turn) link(here, to_id(nr, nc));
            } else if (turn == 1) {
                if (nr == -1) link(here, kTop);
                if (nr == kSide) link(here, kBottom);
            } else {
                if (nc == -1) link(here, kLeft);
                if (nc == kSide) link(here, kRight);
            }
        }
        turn = -turn;
    }

    bool red_won() { return root(kTop) == root(kBottom); }
    bool blue_won() { return root(kLeft) == root(kRight); }
    bool terminal() { return red_won() || blue_won(); }

    bool empty(int row, int col) const { return stone[row][col] == 0; }

    void fill_pair_randomly(int a_row, int a_col, int b_row, int b_col) {
        if (rng() & 1) {
            play(a_row, a_col);
            play(b_row, b_col);
        } else {
            play(b_row, b_col);
            play(a_row, a_col);
        }
    }

    void reinforce_virtual_connections() {
        for (int row = 0; row < kSide; ++row) {
            for (int col = 0; col < kSide; ++col) {
                const int owner = color(row, col);
                if (owner == 0) continue;

                if (inside(row + 1, col - 2) && col - 2 > 0 && owner == color(row + 1, col - 2) &&
                    empty(row + 1, col - 1) && empty(row, col - 1)) {
                    fill_pair_randomly(row + 1, col - 1, row, col - 1);
                }
                if (inside(row + 1, col + 1) && owner == color(row + 1, col + 1) &&
                    empty(row, col + 1) && empty(row + 1, col)) {
                    fill_pair_randomly(row, col + 1, row + 1, col);
                }
                if (inside(row + 2, col - 1) && col - 1 > 0 && owner == color(row + 2, col - 1) &&
                    empty(row + 1, col - 1) && empty(row + 1, col)) {
                    fill_pair_randomly(row + 1, col - 1, row + 1, col);
                }

                if (owner == 1 && row == 1 && col < kSide - 1 && empty(row - 1, col) && empty(row - 1, col + 1)) {
                    fill_pair_randomly(row - 1, col, row - 1, col + 1);
                }
                if (owner == 1 && row == kSide - 2 && col > 0 && empty(row + 1, col - 1) && empty(row + 1, col)) {
                    fill_pair_randomly(row + 1, col, row + 1, col - 1);
                }
                if (owner == -1 && row < kSide - 1 && col == 1 && empty(row, col - 1) && empty(row + 1, col - 1)) {
                    fill_pair_randomly(row, col - 1, row + 1, col - 1);
                }
                if (owner == -1 && row > 0 && col == kSide - 2 && empty(row, col + 1) && empty(row - 1, col + 1)) {
                    fill_pair_randomly(row, col + 1, row - 1, col + 1);
                }
            }
        }
    }
};

Position root_position;
array<array<int, kSide>, kSide> distance_from_stone;
array<int, kCells> queue_row{}, queue_col{}, candidate_id{};

struct SearchTree {
    unique_ptr<int[]> move_id, next_sibling, first_child, visits, wins;
    int cursor = 1;

    SearchTree()
        : move_id(new int[kNodeLimit]()),
          next_sibling(new int[kNodeLimit]()),
          first_child(new int[kNodeLimit]()),
          visits(new int[kNodeLimit]()),
          wins(new int[kNodeLimit]()) {}

    void reset_cursor() { cursor = 1; }

    pair<int, int> choose_child_or_expand() {
        if (first_child[cursor] == 0) {
            for (auto& row : distance_from_stone) row.fill(-1);
            int head = 0, tail = -1, count = 0;

            for (int r = 0; r < kSide; ++r) {
                for (int c = 0; c < kSide; ++c) {
                    if (root_position.color(r, c) == 0) continue;
                    distance_from_stone[r][c] = 0;
                    queue_row[++tail] = r;
                    queue_col[tail] = c;
                }
            }

            while (head <= tail) {
                const int row = queue_row[head];
                const int col = queue_col[head++];
                if (distance_from_stone[row][col] == search_radius) break;
                for (int dir = 0; dir < 6; ++dir) {
                    const int nr = row + kDr[dir];
                    const int nc = col + kDc[dir];
                    if (inside(nr, nc) && distance_from_stone[nr][nc] == -1) {
                        distance_from_stone[nr][nc] = distance_from_stone[row][col] + 1;
                        queue_row[++tail] = nr;
                        queue_col[tail] = nc;
                        candidate_id[++count] = to_id(nr, nc);
                    }
                }
            }

            shuffle(candidate_id.begin() + 1, candidate_id.begin() + count + 1, rng);
            for (int i = 1; i <= count; ++i) {
                const int node = ++generated_nodes;
                move_id[node] = candidate_id[i];
                next_sibling[node] = first_child[cursor];
                first_child[cursor] = node;
            }
            return {move_id[first_child[cursor]], -first_child[cursor]};
        }

        int selected_node = 0;
        int selected_move = 0;
        double best_score = -1e100;
        for (int node = first_child[cursor]; node != 0; node = next_sibling[node]) {
            if (visits[node] == 0) return {move_id[node], node};
            const double exploitation = static_cast<double>(wins[node]) / visits[node];
            const double exploration = kExplore * sqrt(log(max(1, visits[cursor])) / visits[node]);
            const double score = exploitation + exploration;
            if (score > best_score) {
                best_score = score;
                selected_node = node;
                selected_move = move_id[node];
            }
        }
        return {selected_move, selected_node};
    }

    void random_completion() {
        shuffle(cells.begin(), cells.end(), rng);
        for (const auto& [row, col] : cells) {
            if (!root_position.empty(row, col)) {
                if (root_position.terminal()) return;
                continue;
            }

            root_position.play(row, col);
            const int owner = root_position.color(row, col);

            auto try_bridge = [&](int ar, int ac, int br, int bc, int cr, int cc) -> bool {
                if (!inside(ar, ac) || owner != root_position.color(ar, ac)) return false;
                if (!root_position.empty(br, bc) || !root_position.empty(cr, cc)) return false;
                root_position.fill_pair_randomly(br, bc, cr, cc);
                return root_position.terminal();
            };

            if (try_bridge(row - 2, col + 1, row - 1, col, row - 1, col + 1)) return;
            if (try_bridge(row - 1, col - 1, row - 1, col, row, col - 1)) return;
            if (try_bridge(row - 1, col + 2, row - 1, col + 1, row, col + 1)) return;
            if (col - 2 > 0 && try_bridge(row + 1, col - 2, row + 1, col - 1, row, col - 1)) return;
            if (try_bridge(row + 1, col + 1, row, col + 1, row + 1, col)) return;
            if (col - 1 > 0 && try_bridge(row + 2, col - 1, row + 1, col - 1, row + 1, col)) return;

            if (owner == 1 && row == 1 && col < kSide - 1 && root_position.empty(row - 1, col) && root_position.empty(row - 1, col + 1)) {
                root_position.fill_pair_randomly(row - 1, col, row - 1, col + 1);
            }
            if (owner == 1 && row == kSide - 2 && col > 0 && root_position.empty(row + 1, col - 1) && root_position.empty(row + 1, col)) {
                root_position.fill_pair_randomly(row + 1, col, row + 1, col - 1);
            }
            if (owner == -1 && row < kSide - 1 && col == 1 && root_position.empty(row, col - 1) && root_position.empty(row + 1, col - 1)) {
                root_position.fill_pair_randomly(row, col - 1, row + 1, col - 1);
            }
            if (owner == -1 && row > 0 && col == kSide - 2 && root_position.empty(row, col + 1) && root_position.empty(row - 1, col + 1)) {
                root_position.fill_pair_randomly(row, col + 1, row - 1, col + 1);
            }
            if (root_position.terminal()) return;
        }
    }

    void iteration(const Position& input_position) {
        root_position = input_position;
        vector<int> path{1};
        cursor = 1;

        while (!root_position.terminal()) {
            const auto [choice, next_node] = choose_child_or_expand();
            root_position.play(row_of(choice), col_of(choice));
            if (next_node < 0) break;
            path.push_back(next_node);
            cursor = next_node;
        }

        Position bridge_seed = root_position;
        bridge_seed.reinforce_virtual_connections();
        for (int sample = 0; sample < 3; ++sample) {
            root_position = bridge_seed;
            random_completion();
            const int winner = root_position.red_won() ? 1 : -1;
            int side_to_move = input_position.turn;
            for (const int node : path) {
                ++visits[node];
                if (side_to_move != winner) ++wins[node];
                side_to_move = -side_to_move;
            }
        }
    }

    int best_root_move() const {
        int best_move = 0;
        int best_wins = -1;
        for (int node = first_child[1]; node != 0; node = next_sibling[node]) {
            if (wins[node] > best_wins) {
                best_wins = wins[node];
                best_move = move_id[node];
            }
        }
        return best_move;
    }
};

SearchTree mcts;

const int kThirdMoveBook[kCells] = {
-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
-1,-1,-1,80,40,45,80,48,50,45,-1,
-1,35,73,34,40,40,73,73,45,-1,-1,
-1,73,80,80,50,80,61,73,80,45,45,
-1,80,80,45,45,50,42,80,80,50,58,
40,70,45,59,45,80,80,80,80,72,80,
84,85,51,45,45,70,80,80,80,51,80,
59,80,80,51,45,80,70,70,80,72,60,
50,59,45,69,80,80,80,70,70,80,72,
42,71,100,80,80,70,96,70,70,80,40,
-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
};

const int kFourthMoveBook[kCells] = {
-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
40,40,-1,40,40,40,28,40,95,51,62,
71,40,40,40,71,40,96,47,83,62,95,
95,36,95,95,95,50,95,83,83,40,20,
40,71,71,71,71,71,71,62,29,40,40,
92,95,40,71,71,71,40,40,40,96,96,
40,40,59,71,81,40,40,84,40,96,40,
40,93,90,-1,70,71,50,50,40,107,40,
47,40,79,71,71,40,40,84,50,50,40,
40,40,40,40,40,50,50,40,40,40,40,
-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
};

int search_answer(const Position& input_position) {
    while ((clock() - started_at) / static_cast<double>(CLOCKS_PER_SEC) < kThinkSeconds &&
           generated_nodes <= kNodeLimit - 130) {
        mcts.iteration(input_position);
    }
    return mcts.best_root_move();
}

int choose_move(const Position& input_position, bool forced_start) {
    if (forced_start) return to_id(1, 2);

    int occupied = 0;
    for (int row = 0; row < kSide; ++row) {
        for (int col = 0; col < kSide; ++col) {
            occupied += input_position.color(row, col) != 0;
        }
    }

    search_radius = occupied <= 6 ? 3 : 2;
    if (occupied == 1) return to_id(7, 3);

    if (occupied == 2) {
        int peer = 0;
        for (int row = 0; row < kSide; ++row) {
            for (int col = 0; col < kSide; ++col) {
                if (row == 1 && col == 2) continue;
                if (input_position.color(row, col) != 0) peer = to_id(row, col);
            }
        }
        if (kThirdMoveBook[peer] != -1) return kThirdMoveBook[peer];
        search_radius = 20;
        return search_answer(input_position);
    }

    if (occupied == 3) {
        int peer = 0;
        for (int row = 0; row < kSide; ++row) {
            for (int col = 0; col < kSide; ++col) {
                if (row == 1 && col == 2) continue;
                if (row == 7 && col == 3) continue;
                if (input_position.color(row, col) != 0) peer = to_id(row, col);
            }
        }
        if (kFourthMoveBook[peer] != -1) return kFourthMoveBook[peer];
        search_radius = 20;
        return search_answer(input_position);
    }

    return search_answer(input_position);
}

Json::Value make_response(const Position& position, bool forced_start) {
    const int action = choose_move(position, forced_start);
    Json::Value out;
    out["x"] = row_of(action);
    out["y"] = col_of(action);
    return out;
}

}  // namespace

int main() {
    started_at = clock();
    for (int row = 0; row < kSide; ++row) {
        for (int col = 0; col < kSide; ++col) cells.emplace_back(row, col);
    }

    string payload;
    getline(cin, payload);

    Json::Reader reader;
    Json::Value input_json;
    reader.parse(payload, input_json);

    Position game;
    const int finished_turns = input_json["responses"].size();
    for (int turn = 0; turn < finished_turns; ++turn) {
        int x = input_json["requests"][turn]["x"].asInt();
        int y = input_json["requests"][turn]["y"].asInt();
        if (x >= 0 && y >= 0) game.play(x, y);

        x = input_json["responses"][turn]["x"].asInt();
        y = input_json["responses"][turn]["y"].asInt();
        game.play(x, y);
    }

    const int request_x = input_json["requests"][finished_turns]["x"].asInt();
    const int request_y = input_json["requests"][finished_turns]["y"].asInt();
    bool forced_start = false;
    if (request_x >= 0 && request_y >= 0) {
        game.play(request_x, request_y);
    } else {
        forced_start = input_json["requests"][0].isMember("forced_x");
    }

    Json::Value result;
    result["response"] = make_response(game, forced_start);

    Json::FastWriter writer;
    cout << writer.write(result) << '\n';
    return 0;
}
