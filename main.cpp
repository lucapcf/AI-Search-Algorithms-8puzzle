#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <unordered_set>
#include <vector>
using namespace std;

bool is_valid_algorithm(const string &alg) {
    static const unordered_set<string> valid_algorithms = {
        "-bfs", "-idfs", "-astar", "-idastar", "-gbfs"};
    return valid_algorithms.count(alg) > 0;
}

bool is_valid_state_size(const vector<int> &state,
                         vector<int> &expected_sizes) {
    for (int n : expected_sizes) {
        if (state.size() == static_cast<size_t>(n)) {
            expected_sizes = {n};
            return true;
        }
    }
    return false;
}

bool is_valid_puzzle_state(const vector<int> &state, int max_value) {
    unordered_set<int> seen_numbers;

    for (int num : state) {
        if (num < 0 || num > max_value) {
            cerr << "Erro: Numero fora do intervalo permitido: " << num << endl;
            return false;
        }

        if (seen_numbers.find(num) != seen_numbers.end()) {
            cerr << "Erro: Numero repetido: " << num << endl;
            return false;
        }

        seen_numbers.insert(num);
    }

    return true;
}

struct VectorHash {
    size_t operator()(const vector<int> &v) const {
        size_t hash = 0;
        for (int num : v) {
            hash ^=
                std::hash<int>{}(num) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

bool extract_digits(int argc, char *argv[], vector<vector<int>> &states_2d) {
    vector<int> current_state;
    vector<int> expected_sizes = {9, 16};

    for (int i = 2; i < argc; ++i) {
        string arg = argv[i];
        string current_number = "";

        for (char c : arg) {
            if (isdigit(c)) {
                current_number += c;
            } else if (c == ',') {
                if (!current_number.empty()) {
                    current_state.push_back(stoi(current_number));
                    current_number.clear();
                }

                if (!is_valid_state_size(current_state, expected_sizes)) {
                    cerr << "Erro: Estado invalido. Os estados devem ter 9 ou "
                            "16 digitos.\n";
                    return false;
                }

                if (!is_valid_puzzle_state(current_state,
                                           expected_sizes[0] - 1)) {
                    return false;
                }

                states_2d.push_back(current_state);
                current_state.clear();
            } else {
                cerr << "Erro: Caractere invalido: " << c << endl;
                return false;
            }
        }

        if (!current_number.empty()) {
            current_state.push_back(stoi(current_number));
        }
    }

    if (!current_state.empty()) {
        if (!is_valid_state_size(current_state, expected_sizes)) {
            cerr << "Erro: Estado invalido. Os estados devem ter 9 ou 16 "
                    "digitos.\n";
            return false;
        }

        if (!is_valid_puzzle_state(current_state, expected_sizes[0] - 1)) {
            return false;
        }

        states_2d.push_back(current_state);
    }

    return true;
}

int manhattan_distance(const vector<int> &state, int n) {
    int dist = 0;
    int size = n * n;

    for (int i = 0; i < size; i++) {
        int value = state[i];
        if (value != 0) {
            int goal_row = value / n;
            int goal_col = value % n;
            int current_row = i / n;
            int current_col = i % n;
            dist += abs(goal_row - current_row) + abs(goal_col - current_col);
        }
    }
    return dist;
}

bool is_goal(const vector<int> &state, const vector<int> goal) {
    return state == goal;
}

enum Action { NONE, UP, LEFT, RIGHT, DOWN };

class SearchNode {
  public:
    vector<int> state;
    shared_ptr<SearchNode> parent;
    Action action;
    int path_cost;
    int timestamp;
    static int counter;

    SearchNode(const vector<int> &state,
               shared_ptr<SearchNode> parent = nullptr, Action action = NONE,
               int path_cost = 0)
        : state(state), parent(parent), action(action), path_cost(path_cost),
          timestamp(counter++) {}

    static shared_ptr<SearchNode> make_root_node(const vector<int> &state) {
        return make_shared<SearchNode>(state);
    }

    static shared_ptr<SearchNode> make_node(shared_ptr<SearchNode> parent,
                                            Action action,
                                            const vector<int> &state,
                                            int cost) {
        return make_shared<SearchNode>(state, parent, action,
                                       parent->path_cost + cost);
    }
};
int SearchNode::counter = 0;

vector<pair<vector<int>, Action>> get_successors(const vector<int> &state,
                                                 Action last_action) {
    int size = state.size();
    int n = size == 9 ? 3 : 4;

    int pos = find(state.begin(), state.end(), 0) - state.begin();

    vector<pair<vector<int>, Action>> succs;

    vector<pair<int, Action>> directions = {
        {-n, UP}, {-1, LEFT}, {1, RIGHT}, {n, DOWN}};

    for (auto &[delta, action] : directions) {
        int new_pos = pos + delta;
        bool valid_move = false;
        // UP
        if (action == UP && last_action != DOWN && new_pos >= 0) {
            valid_move = true;
        }
        // LEFT
        else if (action == LEFT && last_action != RIGHT && pos % n != 0) {
            valid_move = true;
        }
        // RIGHT
        else if (action == RIGHT && last_action != LEFT && pos % n != n - 1) {
            valid_move = true;
        }
        // DOWN
        else if (action == DOWN && last_action != UP && new_pos < size) {
            valid_move = true;
        }
        if (valid_move) {
            vector<int> new_state = state;
            swap(new_state[pos], new_state[new_pos]);
            succs.push_back({new_state, action});
        }
    }
    return succs;
}

list<Action> extract_path(shared_ptr<SearchNode> node) {
    list<Action> path;
    while (node->parent != nullptr) {
        path.push_front(node->action);
        node = node->parent;
    }
    return path;
}

struct output {
    int expanded_nodes;
    int optimal_solution_length;
    float time;
    float heuristic_avg_value;
    int heuristic_root_value;
};

output bfs(const vector<int> &state) {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchNode::counter = 0;
    static const vector<int> goal_state = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    int heuristic_root_value = manhattan_distance(state, 3);

    if (is_goal(state, goal_state)) {
        auto end_time = std::chrono::high_resolution_clock::now();
        float elapsed_time =
            std::chrono::duration<float>(end_time - start_time).count();
        std::list<Action> empty;
        return {0, 0, elapsed_time, 0.0f, heuristic_root_value};
    }

    deque<shared_ptr<SearchNode>> open;
    open.push_back(SearchNode::make_root_node(state));
    unordered_set<vector<int>, VectorHash> closed;
    closed.insert(state);

    int expanded_nodes = 0;
    while (!open.empty()) {
        shared_ptr<SearchNode> n = open.front();
        open.pop_front();
        auto succs = get_successors(n->state, n->action);
        expanded_nodes++;

        for (auto &[new_state, action] : succs) {
            shared_ptr<SearchNode> new_n =
                SearchNode::make_node(n, action, new_state, 1);
            if (is_goal(new_state, goal_state)) {
                list<Action> path = extract_path(new_n);
                auto end_time = std::chrono::high_resolution_clock::now();
                float elapsed_time =
                    std::chrono::duration<float>(end_time - start_time).count();
                int solution_length = path.size();
                return {expanded_nodes, solution_length, elapsed_time, 0.0f,
                        heuristic_root_value};
            }
            if (closed.find(new_state) == closed.end()) {
                closed.insert(new_state);
                open.push_back(new_n);
            }
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    float elapsed_time =
        std::chrono::duration<float>(end_time - start_time).count();
    // Unobtainable
    return {expanded_nodes, -1, elapsed_time, 0.0f, heuristic_root_value};
}

output gbfs(const vector<int> &state) {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchNode::counter = 0;
    static const vector<int> goal_state = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    int heuristic_root_value = manhattan_distance(state, 3);
    int heuristic_total_value = heuristic_root_value;

    auto cmp = [](const shared_ptr<SearchNode> &a,
                  const shared_ptr<SearchNode> &b) {
        int dist_a = manhattan_distance(a->state, 3);
        int dist_b = manhattan_distance(b->state, 3);
        if (dist_a != dist_b) {
            return dist_a > dist_b;
        }
        if (a->path_cost != b->path_cost) {
            return a->path_cost < b->path_cost;
        }
        return a->timestamp > b->timestamp;
    };

    priority_queue<shared_ptr<SearchNode>, vector<shared_ptr<SearchNode>>,
                   decltype(cmp)>
        open(cmp);
    if (manhattan_distance(state, 3) < numeric_limits<double>::infinity()) {
        open.push(SearchNode::make_root_node(state));
    }
    unordered_set<vector<int>, VectorHash> closed;

    int expanded_nodes = 0;
    while (!open.empty()) {
        shared_ptr<SearchNode> n = open.top();
        open.pop();
        if (closed.find(n->state) == closed.end()) {
            expanded_nodes++;
            heuristic_total_value += manhattan_distance(n->state, 3);
            closed.insert(n->state);
            if (is_goal(n->state, goal_state)) {
                list<Action> path = extract_path(n);
                auto end_time = std::chrono::high_resolution_clock::now();
                float elapsed_time =
                    std::chrono::duration<float>(end_time - start_time).count();
                int solution_length = path.size();
                return {expanded_nodes, solution_length, elapsed_time,
                        (float)heuristic_total_value / expanded_nodes,
                        heuristic_root_value};
            }

            auto succs = get_successors(n->state, n->action);
            for (auto &[new_state, action] : succs) {
                if (closed.find(new_state) == closed.end() &&
                    manhattan_distance(new_state, 3) <
                        numeric_limits<double>::infinity()) {
                    shared_ptr<SearchNode> new_n =
                        SearchNode::make_node(n, action, new_state, 1);
                    open.push(new_n);
                }
            }
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    float elapsed_time =
        std::chrono::duration<float>(end_time - start_time).count();
    // Unobtainable
    return {expanded_nodes, -1, elapsed_time,
            (float)heuristic_total_value / expanded_nodes,
            heuristic_root_value};
}

output astar(const vector<int> &state) {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchNode::counter = 0;
    static const vector<int> goal_state = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    int heuristic_root_value = manhattan_distance(state, 3);
    int heuristic_total_value = heuristic_root_value;

    auto cmp = [](const shared_ptr<SearchNode> &a,
                  const shared_ptr<SearchNode> &b) {
        int h_a = manhattan_distance(a->state, 3);
        int h_b = manhattan_distance(b->state, 3);
        int f_a = h_a + a->path_cost;
        int f_b = h_b + b->path_cost;
        if (f_a != f_b) {
            return f_a > f_b;
        }
        if (h_a != h_b) {
            return h_a > h_b;
        }
        return a->timestamp > b->timestamp;
    };

    priority_queue<shared_ptr<SearchNode>, vector<shared_ptr<SearchNode>>,
                   decltype(cmp)>
        open(cmp);
    if (manhattan_distance(state, 3) < numeric_limits<double>::infinity()) {
        open.push(SearchNode::make_root_node(state));
    }
    unordered_set<vector<int>, VectorHash> closed;

    int expanded_nodes = 0;
    while (!open.empty()) {
        shared_ptr<SearchNode> n = open.top();
        open.pop();
        if (closed.find(n->state) == closed.end()) {
            expanded_nodes++;
            heuristic_total_value += manhattan_distance(n->state, 3);
            closed.insert(n->state);
            if (is_goal(n->state, goal_state)) {
                list<Action> path = extract_path(n);
                auto end_time = std::chrono::high_resolution_clock::now();
                float elapsed_time =
                    std::chrono::duration<float>(end_time - start_time).count();
                int solution_length = path.size();
                return {expanded_nodes, solution_length, elapsed_time,
                        (float)heuristic_total_value / expanded_nodes,
                        heuristic_root_value};
            }

            auto succs = get_successors(n->state, n->action);
            for (auto &[new_state, action] : succs) {
                if (closed.find(new_state) == closed.end() &&
                    manhattan_distance(new_state, 3) <
                        numeric_limits<double>::infinity()) {
                    shared_ptr<SearchNode> new_n =
                        SearchNode::make_node(n, action, new_state, 1);
                    open.push(new_n);
                }
            }
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    float elapsed_time =
        std::chrono::duration<float>(end_time - start_time).count();
    // Unobtainable
    return {expanded_nodes, -1, elapsed_time,
            (float)heuristic_total_value / expanded_nodes,
            heuristic_root_value};
}

stack<Action> depth_limited_search(const vector<int> &state, int depth_limit,
                                   Action last_action, int &expanded_nodes) {
    static const vector<int> goal_state = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    if (is_goal(state, goal_state)) {
        return stack<Action>();
    }
    expanded_nodes++;
    if (depth_limit > 0) {
        auto succs = get_successors(state, last_action);
        for (auto &[new_state, action] : succs) {
            stack<Action> solution = depth_limited_search(
                new_state, depth_limit - 1, action, expanded_nodes);
            if (solution.empty() || solution.top() != NONE) {
                solution.push(action);
                return solution;
            }
        }
    }
    return stack<Action>({NONE});
}

output idfs(const vector<int> &state) {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchNode::counter = 0;
    static const vector<int> goal_state = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    int heuristic_root_value = manhattan_distance(state, 3);
    int expanded_nodes = 0;

    for (int depth_limit = 0;; depth_limit++) {
        stack<Action> solution =
            depth_limited_search(state, depth_limit, NONE, expanded_nodes);
        if (solution.empty() || solution.top() != NONE) {
            auto end_time = std::chrono::high_resolution_clock::now();
            float elapsed_time =
                std::chrono::duration<float>(end_time - start_time).count();
            return {expanded_nodes, depth_limit, elapsed_time, 0.0f,
                    heuristic_root_value};
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc < 10) {
        cerr << "Uso: -<algoritmo> <estado1>,<estado2>,...\n";
        return 1;
    }

    string algorithm = argv[1];
    if (!is_valid_algorithm(algorithm)) {
        cerr << "Erro: Algoritmo invalido.\n";
        return 1;
    }

    vector<vector<int>> states_2d;

    if (!extract_digits(argc, argv, states_2d)) {
        return 1;
    }

    cout << "Algoritmo: " << algorithm << "\n";
    cout << "Estados:\n";
    for (const auto &state : states_2d) {
        for (const int &num : state) {
            cout << num << " ";
        }
        cout << "\n";
    }

    if (algorithm == "-bfs") {
        for (const auto &state : states_2d) {
            output result = bfs(state);

            cout << result.expanded_nodes << ","
                 << result.optimal_solution_length << "," << result.time << ","
                 << result.heuristic_avg_value << ","
                 << result.heuristic_root_value << "\n";
        }
    } else if (algorithm == "-gbfs") {
        for (const auto &state : states_2d) {
            output result = gbfs(state);

            cout << result.expanded_nodes << ","
                 << result.optimal_solution_length << "," << result.time << ","
                 << result.heuristic_avg_value << ","
                 << result.heuristic_root_value << "\n";
        }
    } else if (algorithm == "-astar") {
        for (const auto &state : states_2d) {
            output result = astar(state);

            cout << result.expanded_nodes << ","
                 << result.optimal_solution_length << "," << result.time << ","
                 << result.heuristic_avg_value << ","
                 << result.heuristic_root_value << "\n";
        }
    } else if (algorithm == "-idfs") {
        for (const auto &state : states_2d) {
            output result = idfs(state);

            cout << result.expanded_nodes << ","
                 << result.optimal_solution_length << "," << result.time << ","
                 << result.heuristic_avg_value << ","
                 << result.heuristic_root_value << "\n";
        }
    } else {
        cout << "O algoritmo selecionado ainda não foi implementado.\n";
    }

    return 0;
}
