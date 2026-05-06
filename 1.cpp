#pragma GCC optimize("O3,unroll-loops")
#include <bits/stdc++.h>

using namespace std;

constexpr int B_SIZE = 11;
constexpr int CELL_COUNT = B_SIZE * B_SIZE;
constexpr double EXPLORE_RATE = 0.40;
constexpr double PRIOR_WEIGHT = 2.8;
constexpr double TIME_LIMIT_SEC = 0.90;
constexpr int POOL_CAPACITY = 600005;

unsigned init_seed() {
    if (const char* env_seed = getenv("HEX_SEED")) {
        char* end_ptr = nullptr;
        unsigned long parsed = strtoul(env_seed, &end_ptr, 10);
        if (end_ptr != env_seed && *end_ptr == '\0') {
            return static_cast<unsigned>(parsed);
        }
    }
    return static_cast<unsigned>(chrono::steady_clock::now().time_since_epoch().count());
}

static mt19937 rand_gen(init_seed());

const int DIR_R[6] = {0, 1, 1, 0, -1, -1};
const int DIR_C[6] = {1, 0, -1, -1, 0, 1};
const int BRG_R[6] = {-1, 1, 2, 1, -1, -2};
const int BRG_C[6] = {2, 1, -1, -2, -1, 1};

inline bool in_board(int r, int c) {
    return static_cast<unsigned>(r) < B_SIZE && static_cast<unsigned>(c) < B_SIZE;
}

inline int cell_id(int r, int c) {
    return r * B_SIZE + c;
}

struct DisjointSet {
    int parent_node[CELL_COUNT], set_size[CELL_COUNT], edge_mask[CELL_COUNT];

    inline void clear_all() {
        for (int i = 0; i < CELL_COUNT; ++i) {
            parent_node[i] = i;
            set_size[i] = 1;
            edge_mask[i] = 0;
        }
    }

    inline int find_root(int curr) {
        int root = curr;
        while (parent_node[root] != root) root = parent_node[root];
        while (parent_node[curr] != root) {
            int tmp = parent_node[curr];
            parent_node[curr] = root;
            curr = tmp;
        }
        return root;
    }

    inline bool merge_sets(int a, int b) {
        int root_a = find_root(a), root_b = find_root(b);
        if (root_a != root_b) {
            if (set_size[root_a] > set_size[root_b]) swap(root_a, root_b);
            parent_node[root_a] = root_b;
            set_size[root_b] += set_size[root_a];
            edge_mask[root_b] |= edge_mask[root_a];
        }
        return edge_mask[root_b] == 3;
    }
};

struct HexBoard {
    int8_t grid[CELL_COUNT];
    DisjointSet dsu;

    HexBoard() {
        memset(grid, 0, sizeof(grid));
        dsu.clear_all();
    }

    inline int at(int r, int c) const {
        return grid[cell_id(r, c)];
    }

    inline bool empty(int id) const {
        return grid[id] == 0;
    }

    bool put_piece_id(int id, int p_color) {
        grid[id] = static_cast<int8_t>(p_color);
        int r = id / B_SIZE, c = id % B_SIZE;

        if (p_color == 1) {
            if (r == 0) dsu.edge_mask[id] |= 1;
            if (r == B_SIZE - 1) dsu.edge_mask[id] |= 2;
        } else {
            if (c == 0) dsu.edge_mask[id] |= 1;
            if (c == B_SIZE - 1) dsu.edge_mask[id] |= 2;
        }
        if (dsu.edge_mask[id] == 3) return true;

        for (int d = 0; d < 6; ++d) {
            int nr = r + DIR_R[d], nc = c + DIR_C[d];
            if (in_board(nr, nc)) {
                int nid = cell_id(nr, nc);
                if (grid[nid] == p_color && dsu.merge_sets(id, nid)) return true;
            }
        }
        return false;
    }

    inline bool put_piece(int r, int c, int p_color) {
        return put_piece_id(cell_id(r, c), p_color);
    }
};

double eval_position_id(int id, const HexBoard& b, int self_color) {
    int r = id / B_SIZE, c = id % B_SIZE;
    double base_val = 10.0 - abs(r - 5) - abs(c - 5);

    for (int k = 0; k < 6; ++k) {
        int adj_r = r + DIR_R[k], adj_c = c + DIR_C[k];
        if (in_board(adj_r, adj_c)) {
            int val = b.at(adj_r, adj_c);
            if (val == self_color) base_val += 30.0;
            else if (val == -self_color) base_val += 20.0;
        }

        int bg_r = r + BRG_R[k], bg_c = c + BRG_C[k];
        if (in_board(bg_r, bg_c)) {
            int val = b.at(bg_r, bg_c);
            if (val == self_color) base_val += 60.0;
            else if (val == -self_color) base_val += 30.0;
        }
    }

    base_val += (self_color == 1 ? (5.0 - abs(r - 5)) : (5.0 - abs(c - 5))) * 5.0;
    return base_val;
}

inline double normalized_prior(double raw_score) {
    return clamp((raw_score + 50.0) / 250.0, 0.0, 1.0);
}

struct TreeNode {
    int8_t r_idx, c_idx, player_color;
    int visit_count;
    double win_score, prior_val;
    int p_node, head_child, right_bro;
    uint8_t pend_id[CELL_COUNT];
    int pend_size;
};

struct Action {
    double score;
    uint8_t id;

    bool operator<(const Action& other) const {
        return score < other.score;
    }
};

TreeNode* tree_nodes;
int total_nodes = 0;

int spawn_node(int move_r, int move_c, int move_col, int parent, const HexBoard& b, int root_color) {
    int cur_id = ++total_nodes;
    TreeNode& nd = tree_nodes[cur_id];
    nd.r_idx = static_cast<int8_t>(move_r);
    nd.c_idx = static_cast<int8_t>(move_c);
    nd.player_color = static_cast<int8_t>(move_col);
    nd.visit_count = 0;
    nd.win_score = 0.0;
    nd.p_node = parent;
    nd.head_child = 0;
    nd.right_bro = 0;
    nd.pend_size = 0;

    bool has_piece = false;
    bool near_piece[CELL_COUNT] = {};

    for (int id = 0; id < CELL_COUNT; ++id) {
        if (b.grid[id] == 0) continue;
        has_piece = true;
        int r = id / B_SIZE, c = id % B_SIZE;
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                int nr = r + dy, nc = c + dx;
                if (in_board(nr, nc)) {
                    int dist = max(max(abs(dy), abs(dx)), abs(dy + dx));
                    if (dist <= 2) near_piece[cell_id(nr, nc)] = true;
                }
            }
        }
    }

    Action actions[CELL_COUNT];
    int action_count = 0;

    for (int id = 0; id < CELL_COUNT; ++id) {
        if (!b.empty(id)) continue;
        if (!has_piece || near_piece[id]) {
            actions[action_count++] = {
                eval_position_id(id, b, root_color),
                static_cast<uint8_t>(id)
            };
        }
    }

    if (action_count == 0) {
        for (int id = 0; id < CELL_COUNT; ++id) {
            if (b.empty(id)) {
                actions[action_count++] = {
                    eval_position_id(id, b, root_color),
                    static_cast<uint8_t>(id)
                };
            }
        }
    }

    sort(actions, actions + action_count);
    for (int i = 0; i < action_count; ++i) {
        nd.pend_id[nd.pend_size] = actions[i].id;
        ++nd.pend_size;
    }

    nd.prior_val = (move_r == -1) ? 0.0 : normalized_prior(eval_position_id(cell_id(move_r, move_c), b, root_color));
    return cur_id;
}

inline int play_and_flip(HexBoard& b, int id, int& current_player) {
    if (b.put_piece_id(id, current_player)) return current_player;
    current_player = -current_player;
    return 0;
}

int fast_playout(HexBoard curr_b, int current_player) {
    int empties[CELL_COUNT], e_cnt = 0;
    for (int id = 0; id < CELL_COUNT; ++id) {
        if (curr_b.empty(id)) empties[e_cnt++] = id;
    }

    shuffle(empties, empties + e_cnt, rand_gen);

    auto try_pair = [&](int id1, int id2) -> int {
        if (rand_gen() & 1) swap(id1, id2);
        int winner = play_and_flip(curr_b, id1, current_player);
        if (winner) return winner;
        return play_and_flip(curr_b, id2, current_player);
    };

    for (int idx = 0; idx < e_cnt; ++idx) {
        int id = empties[idx];
        if (!curr_b.empty(id)) continue;

        int r = id / B_SIZE, c = id % B_SIZE;
        int winner = play_and_flip(curr_b, id, current_player);
        if (winner) return winner;

#define REPAIR_BRIDGE(br, bc, m1r, m1c, m2r, m2c) \
        if (in_board((br), (bc)) && curr_b.grid[id] == curr_b.grid[cell_id((br), (bc))]) { \
            int m1 = cell_id((m1r), (m1c)), m2 = cell_id((m2r), (m2c)); \
            if (curr_b.empty(m1) && curr_b.empty(m2)) { \
                int res = try_pair(m1, m2); \
                if (res) return res; \
            } \
        }

        REPAIR_BRIDGE(r - 2, c + 1, r - 1, c, r - 1, c + 1)
        REPAIR_BRIDGE(r - 1, c - 1, r - 1, c, r, c - 1)
        REPAIR_BRIDGE(r - 1, c + 2, r - 1, c + 1, r, c + 1)
        REPAIR_BRIDGE(r + 1, c - 2, r + 1, c - 1, r, c - 1)
        REPAIR_BRIDGE(r + 1, c + 1, r, c + 1, r + 1, c)
        REPAIR_BRIDGE(r + 2, c - 1, r + 1, c - 1, r + 1, c)

#undef REPAIR_BRIDGE

        if (curr_b.grid[id] == 1) {
            if (r == 1 && c < 10) {
                int a = cell_id(r - 1, c), b = cell_id(r - 1, c + 1);
                if (curr_b.empty(a) && curr_b.empty(b)) {
                    int res = try_pair(a, b);
                    if (res) return res;
                }
            }
            if (r == 9 && c > 0) {
                int a = cell_id(r + 1, c), b = cell_id(r + 1, c - 1);
                if (curr_b.empty(a) && curr_b.empty(b)) {
                    int res = try_pair(a, b);
                    if (res) return res;
                }
            }
        } else if (curr_b.grid[id] == -1) {
            if (c == 1 && r < 10) {
                int a = cell_id(r, c - 1), b = cell_id(r + 1, c - 1);
                if (curr_b.empty(a) && curr_b.empty(b)) {
                    int res = try_pair(a, b);
                    if (res) return res;
                }
            }
            if (c == 9 && r > 0) {
                int a = cell_id(r, c + 1), b = cell_id(r - 1, c + 1);
                if (curr_b.empty(a) && curr_b.empty(b)) {
                    int res = try_pair(a, b);
                    if (res) return res;
                }
            }
        }
    }

    return 0;
}

pair<int, int> execute_mcts(HexBoard& base_state, int my_side) {
    for (int id = 0; id < CELL_COUNT; ++id) {
        if (base_state.empty(id)) {
            HexBoard attack = base_state;
            if (attack.put_piece_id(id, my_side)) return {id / B_SIZE, id % B_SIZE};

            HexBoard defend = base_state;
            if (defend.put_piece_id(id, -my_side)) return {id / B_SIZE, id % B_SIZE};
        }
    }

    total_nodes = 0;
    int root_idx = spawn_node(-1, -1, -my_side, 0, base_state, my_side);
    auto started_at = chrono::steady_clock::now();

    while (chrono::duration<double>(chrono::steady_clock::now() - started_at).count() < TIME_LIMIT_SEC &&
           total_nodes < POOL_CAPACITY - 150) {
        int track_node = root_idx, active_side = my_side;
        HexBoard sim_board = base_state;

        while (tree_nodes[track_node].pend_size == 0 && tree_nodes[track_node].head_child != 0) {
            double highest_ucb = -1e19;
            int selected_child = 0;
            const double parent_log = log(tree_nodes[track_node].visit_count + 1.0);

            for (int ch = tree_nodes[track_node].head_child; ch != 0; ch = tree_nodes[ch].right_bro) {
                double visits = tree_nodes[ch].visit_count + 1e-7;
                double final_uct = (tree_nodes[ch].win_score / visits) +
                                   EXPLORE_RATE * sqrt(parent_log / visits) +
                                   (PRIOR_WEIGHT * tree_nodes[ch].prior_val) / (tree_nodes[ch].visit_count + 1);
                if (final_uct > highest_ucb) {
                    highest_ucb = final_uct;
                    selected_child = ch;
                }
            }

            track_node = selected_child;
            int move_id = cell_id(tree_nodes[track_node].r_idx, tree_nodes[track_node].c_idx);
            sim_board.put_piece_id(move_id, tree_nodes[track_node].player_color);
            active_side = -tree_nodes[track_node].player_color;
        }

        if (tree_nodes[track_node].pend_size > 0) {
            --tree_nodes[track_node].pend_size;
            int next_id = tree_nodes[track_node].pend_id[tree_nodes[track_node].pend_size];
            int nr = next_id / B_SIZE, nc = next_id % B_SIZE;
            sim_board.put_piece_id(next_id, active_side);

            int new_ch = spawn_node(nr, nc, active_side, track_node, sim_board, my_side);
            tree_nodes[new_ch].right_bro = tree_nodes[track_node].head_child;
            tree_nodes[track_node].head_child = new_ch;
            track_node = new_ch;
            active_side = -active_side;
        }

        int match_result = fast_playout(sim_board, active_side);

        while (track_node != 0) {
            ++tree_nodes[track_node].visit_count;
            if (match_result == my_side) tree_nodes[track_node].win_score += 1.0;
            else if (match_result != 0) tree_nodes[track_node].win_score -= 0.6;
            track_node = tree_nodes[track_node].p_node;
        }
    }

    pair<int, int> best_step = {-1, -1};
    int most_visits = -1;
    for (int ch = tree_nodes[root_idx].head_child; ch != 0; ch = tree_nodes[ch].right_bro) {
        if (tree_nodes[ch].visit_count > most_visits) {
            most_visits = tree_nodes[ch].visit_count;
            best_step = {tree_nodes[ch].r_idx, tree_nodes[ch].c_idx};
        }
    }
    return best_step;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    tree_nodes = new TreeNode[POOL_CAPACITY];

    int total_turns, op_x, op_y;
    if (!(cin >> total_turns)) {
        delete[] tree_nodes;
        return 0;
    }

    HexBoard current_game;
    for (int t = 0; t < total_turns - 1; ++t) {
        cin >> op_x >> op_y;
        if (op_x != -1) current_game.put_piece(op_x, op_y, -1);

        cin >> op_x >> op_y;
        if (op_x != -1) current_game.put_piece(op_x, op_y, 1);
    }

    cin >> op_x >> op_y;
    int team_flag = (op_x == -1 && total_turns == 1) ? 1 : -1;

    if (total_turns == 1 && team_flag == 1) {
        cout << "1 2\n";
        delete[] tree_nodes;
        return 0;
    }

    if (op_x != -1) current_game.put_piece(op_x, op_y, -team_flag);

    pair<int, int> decision = execute_mcts(current_game, team_flag);
    cout << decision.first << ' ' << decision.second << '\n';

    delete[] tree_nodes;
    return 0;
}
