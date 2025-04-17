#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib> // for atoi or atof
#include <cstring>
#include <deque>
#include <fstream>
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

const long long int goal_state3 = 36344967696;
const long long int goal_state4 = 18364758544493064720;
const std::set<int> expected_sizes = {9, 16};
int state_size = 0;

bool is_valid_algorithm(const string &alg) {
    static const unordered_set<string> valid_algorithms = {
        "-bfs", "-idfs", "-astar", "-idastar", "-gbfs"};
    return valid_algorithms.count(alg) > 0;
}

int is_valid_state_size(const vector<int> &state) {
    for (int n : expected_sizes) {
        if (state.size() == static_cast<size_t>(n)) {
            return n;
        }
    }
    return -1;
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

bool extract_digits(int argc, char *argv[], vector<vector<int>> &states_2d) {
    vector<int> state;

    for (int i = 2; i < argc; i++) {
        string arg = argv[i];
        string current_number = "";

        for (char c : arg) {
            if (isdigit(c)) {
                current_number += c;
            } else if (c == ',') {
                if (!current_number.empty()) {
                    state.push_back(stoi(current_number));
                    current_number.clear();
                }

                if (is_valid_state_size(state) == -1 ||
                    (state.size() != static_cast<std::size_t>(state_size) &&
                     state_size != 0)) {
                    cerr << "Erro: Estado invalido. Os estados devem ter 9 ou "
                            "16 digitos.\n";
                    return false;
                } else {
                    state_size = state.size();
                }

                if (!is_valid_puzzle_state(state, state_size - 1)) {
                    return false;
                }

                states_2d.push_back(state);
                state.clear();
            } else {
                cerr << "Erro: Caractere invalido: " << c << endl;
                return false;
            }
        }

        if (!current_number.empty()) {
            state.push_back(stoi(current_number));
        }
    }

    if (!state.empty()) {
        if (is_valid_state_size(state) == -1 ||
            (state.size() != static_cast<std::size_t>(state_size) &&
             state_size != 0)) {
            cerr << "Erro: Estado invalido. Os estados devem ter 9 ou "
                    "16 digitos.\n";
            return false;
        } else {
            state_size = state.size();
        }

        if (!is_valid_puzzle_state(state, state_size - 1)) {
            return false;
        }

        states_2d.push_back(state);
    }

    return true;
}

bool extract_digits_from_stream(istream &in, vector<vector<int>> &states_2d) {
    string line;
    while (std::getline(in, line)) {
        if (line.empty())
            continue;

        std::istringstream iss(line);
        vector<int> state;
        string token;
        while (iss >> token) {
            try {
                int number = std::stoi(token);
                state.push_back(number);
            } catch (const std::invalid_argument &) {
                std::cerr << "Aviso: Token inválido ignorado: " << token
                          << std::endl;
                continue;
            }
        }
        if (!state.empty()) {
            if (is_valid_state_size(state) == -1 ||
                (state.size() != static_cast<std::size_t>(state_size) &&
                 state_size != 0)) {
                cerr << "Erro: Estado invalido. Os estados devem ter 9 ou 16 "
                        "digitos.\n";
                return false;
            } else {
                state_size = state.size();
            }

            if (!is_valid_puzzle_state(state, state_size - 1)) {
                return false;
            }

            states_2d.push_back(state);
        }
    }

    if (states_2d.empty()) {
        cerr << "Erro: Nenhum estado foi lido do stream.\n";
        return false;
    }

    return true;
}

int manhattan_distance(long long int state, int n) {
    int dist = 0;
    int size = n * n;

    for (int i = 0; i < size; i++) {
        int value = (state >> (4 * i)) & 0xF;
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

bool is_goal(long long int state, int size) {
    if (size == 3)
        return state == goal_state3;
    else if (size == 4)
        return state == goal_state4;
    return false;
}

enum Action { NONE, UP, LEFT, RIGHT, DOWN };

class SearchNode {
  public:
    long long int state;
    shared_ptr<SearchNode> parent;
    Action action;
    int path_cost;
    int timestamp;
    static int counter;

    SearchNode(long long int state, shared_ptr<SearchNode> parent = nullptr,
               Action action = NONE, int path_cost = 0)
        : state(state), parent(parent), action(action), path_cost(path_cost),
          timestamp(counter++) {}

    static shared_ptr<SearchNode> make_root_node(long long int state) {
        return make_shared<SearchNode>(state);
    }

    static shared_ptr<SearchNode> make_node(shared_ptr<SearchNode> parent,
                                            Action action, long long int state,
                                            int cost) {
        return make_shared<SearchNode>(state, parent, action,
                                       parent->path_cost + cost);
    }
};
int SearchNode::counter = 0;

int get_tile(long long state, int pos) { return (state >> (4 * pos)) & 0xF; }

long long int set_tile(long long state, int pos, int value) {
    long long mask = 0xFLL << (4 * pos);
    state &= ~mask;
    state |= (static_cast<long long>(value) << (4 * pos));
    return state;
}

int zero_position(long long state, int n) {
    for (int i = 0; i < n * n; i++) {
        if (get_tile(state, i) == 0) {
            return i;
        }
    }
    return -1;
}

long long int swap_tiles(long long int state, int pos1, int pos2) {
    int tile1 = get_tile(state, pos1);
    int tile2 = get_tile(state, pos2);
    state = set_tile(state, pos1, tile2);
    state = set_tile(state, pos2, tile1);
    return state;
}

long long int encode_state(const vector<int> &state) {
    long long int encoded = 0;
    for (size_t i = 0; i < state.size(); ++i) {
        encoded |= (static_cast<long long int>(state[i]) << (4 * i));
    }
    return encoded;
}

vector<pair<long long int, Action>> get_successors(long long int state,
                                                   Action last_action, int n) {
    int pos = zero_position(state, n);

    vector<pair<long long int, Action>> succs;

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
        else if (action == DOWN && last_action != UP && new_pos < n * n) {
            valid_move = true;
        }
        if (valid_move) {
            long long int new_state = state;
            new_state = swap_tiles(state, pos, new_pos);
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

output bfs(long long int state, int size) {
    auto start_time = std::chrono::high_resolution_clock::now();

    int heuristic_root_value = manhattan_distance(state, size);

    if (is_goal(state, size)) {
        auto end_time = std::chrono::high_resolution_clock::now();
        float elapsed_time =
            std::chrono::duration<float>(end_time - start_time).count();
        std::list<Action> empty;
        return {0, 0, elapsed_time, 0.0f, heuristic_root_value};
    }

    deque<shared_ptr<SearchNode>> open;
    open.push_back(SearchNode::make_root_node(state));
    unordered_set<long long int> closed;
    closed.insert(state);

    int expanded_nodes = 0;
    while (!open.empty()) {
        shared_ptr<SearchNode> n = open.front();
        open.pop_front();
        auto succs = get_successors(n->state, n->action, size);
        expanded_nodes++;

        for (auto &[new_state, action] : succs) {
            shared_ptr<SearchNode> new_n =
                SearchNode::make_node(n, action, new_state, 1);
            if (is_goal(new_state, size)) {
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
    return {expanded_nodes, -1, elapsed_time, 0.0f, heuristic_root_value};
}

output gbfs(long long int state, int size) {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchNode::counter = 0;
    int heuristic_root_value = manhattan_distance(state, size);
    int heuristic_total_value = 0;

    auto cmp = [size](const shared_ptr<SearchNode> &a,
                      const shared_ptr<SearchNode> &b) {
        int dist_a = manhattan_distance(a->state, size);
        int dist_b = manhattan_distance(b->state, size);
        if (dist_a != dist_b) {
            return dist_a > dist_b;
        }
        if (a->path_cost != b->path_cost) {
            return a->path_cost < b->path_cost;
        }
        return a->timestamp < b->timestamp;
    };

    priority_queue<shared_ptr<SearchNode>, vector<shared_ptr<SearchNode>>,
                   decltype(cmp)>
        open(cmp);
    if (manhattan_distance(state, size) < numeric_limits<double>::infinity()) {
        open.push(SearchNode::make_root_node(state));
    }
    unordered_set<long long int> closed;

    int expanded_nodes = 0;
    while (!open.empty()) {
        shared_ptr<SearchNode> n = open.top();
        open.pop();
        if (closed.find(n->state) == closed.end()) {
            closed.insert(n->state);
            if (is_goal(n->state, size)) {
                list<Action> path = extract_path(n);
                auto end_time = std::chrono::high_resolution_clock::now();
                float elapsed_time =
                    std::chrono::duration<float>(end_time - start_time).count();
                int solution_length = path.size();
                return {expanded_nodes, solution_length, elapsed_time,
                        expanded_nodes != 0
                            ? (float)heuristic_total_value / expanded_nodes
                            : 0,
                        heuristic_root_value};
            }

            auto succs = get_successors(n->state, n->action, size);
            expanded_nodes++;
            for (auto &[new_state, action] : succs) {
                if (closed.find(new_state) == closed.end() &&
                    manhattan_distance(new_state, size) <
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
            expanded_nodes != 0 ? (float)heuristic_total_value / expanded_nodes
                                : 0,
            heuristic_root_value};
}

output astar(long long int state, int size) {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchNode::counter = 0;
    int heuristic_root_value = manhattan_distance(state, size);
    int heuristic_total_value = 0;

    auto cmp = [size](const shared_ptr<SearchNode> &a,
                      const shared_ptr<SearchNode> &b) {
        int h_a = manhattan_distance(a->state, size);
        int h_b = manhattan_distance(b->state, size);
        int f_a = h_a + a->path_cost;
        int f_b = h_b + b->path_cost;
        if (f_a != f_b) {
            return f_a > f_b;
        }
        if (h_a != h_b) {
            return h_a > h_b;
        }
        return a->timestamp < b->timestamp;
    };

    priority_queue<shared_ptr<SearchNode>, vector<shared_ptr<SearchNode>>,
                   decltype(cmp)>
        open(cmp);
    if (manhattan_distance(state, size) < numeric_limits<double>::infinity()) {
        open.push(SearchNode::make_root_node(state));
    }
    unordered_set<long long int> closed;

    int expanded_nodes = 0;
    while (!open.empty()) {
        shared_ptr<SearchNode> n = open.top();
        open.pop();
        if (closed.find(n->state) == closed.end()) {
            closed.insert(n->state);
            if (is_goal(n->state, size)) {
                list<Action> path = extract_path(n);
                auto end_time = std::chrono::high_resolution_clock::now();
                float elapsed_time =
                    std::chrono::duration<float>(end_time - start_time).count();
                int solution_length = path.size();
                return {expanded_nodes, solution_length, elapsed_time,
                        expanded_nodes != 0
                            ? (float)heuristic_total_value / expanded_nodes
                            : 0,
                        heuristic_root_value};
            }
            expanded_nodes++;
            auto succs = get_successors(n->state, n->action, size);
            for (auto &[new_state, action] : succs) {
                if (closed.find(new_state) == closed.end() &&
                    manhattan_distance(new_state, size) <
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
    return {expanded_nodes, -1, elapsed_time,
            expanded_nodes != 0 ? (float)heuristic_total_value / expanded_nodes
                                : 0,
            heuristic_root_value};
}

stack<Action> depth_limited_search(long long int state, int depth_limit,
                                   Action last_action, int &expanded_nodes,
                                   int size) {
    if (is_goal(state, size)) {
        return stack<Action>();
    }
    if (depth_limit > 0) {
        auto succs = get_successors(state, last_action, size);
        expanded_nodes++;
        for (auto &[new_state, action] : succs) {
            stack<Action> solution = depth_limited_search(
                new_state, depth_limit - 1, action, expanded_nodes, size);
            if (solution.empty() || solution.top() != NONE) {
                solution.push(action);
                return solution;
            }
        }
    }
    return stack<Action>({NONE});
}

output idfs(long long int state, int size) {
    auto start_time = std::chrono::high_resolution_clock::now();

    int heuristic_root_value = manhattan_distance(state, size);
    int expanded_nodes = 0;

    for (int depth_limit = 0;; depth_limit++) {
        stack<Action> solution = depth_limited_search(state, depth_limit, NONE,
                                                      expanded_nodes, size);
        if (solution.empty() || solution.top() != NONE) {
            auto end_time = std::chrono::high_resolution_clock::now();
            float elapsed_time =
                std::chrono::duration<float>(end_time - start_time).count();
            return {expanded_nodes, depth_limit, elapsed_time, 0.0f,
                    heuristic_root_value};
        }
    }
}

pair<int, list<Action>> recursive_search(shared_ptr<SearchNode> n, int f_limit,
                                         Action last_action,
                                         int &expanded_nodes, int size) {
    int h_value = manhattan_distance(n->state, size);

    int f = h_value + n->path_cost;

    if (f > f_limit) {
        return make_pair(f, list<Action>());
    }

    if (is_goal(n->state, size)) {
        return make_pair(0, extract_path(n));
    }

    expanded_nodes++;
    int next_limit = numeric_limits<int>::max();

    auto succs = get_successors(n->state, last_action, size);
    for (auto &[new_state, action] : succs) {
        int h = manhattan_distance(new_state, size);

        if (h < numeric_limits<int>::max()) {
            shared_ptr<SearchNode> new_node =
                SearchNode::make_node(n, action, new_state, 1);
            int rec_limit;
            list<Action> solution;
            tie(rec_limit, solution) = recursive_search(
                new_node, f_limit, action, expanded_nodes, size);
            if (!solution.empty() && solution.back() != NONE) {
                return make_pair(f_limit, solution);
            }
            next_limit = min(next_limit, rec_limit);
        }
    }
    return make_pair(next_limit, list<Action>({NONE}));
}

output idastar(long long state, int size) {
    auto start_time = std::chrono::high_resolution_clock::now();

    int heuristic_root_value = manhattan_distance(state, size);
    int heuristic_total_value = 0;
    int expanded_nodes = 0;

    shared_ptr<SearchNode> n0 = SearchNode::make_root_node(state);
    int f_limit = heuristic_root_value;

    while (f_limit < numeric_limits<int>::max()) {
        list<Action> solution;
        int new_f_limit;

        tie(new_f_limit, solution) =
            recursive_search(n0, f_limit, NONE, expanded_nodes, size);

        if (solution.empty() || solution.back() != NONE) {
            auto end_time = std::chrono::high_resolution_clock::now();
            float elapsed_time =
                std::chrono::duration<float>(end_time - start_time).count();
            return {expanded_nodes, f_limit, elapsed_time,
                    expanded_nodes != 0
                        ? (float)heuristic_total_value / expanded_nodes
                        : 0,
                    heuristic_root_value};
        }

        f_limit = new_f_limit;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    float elapsed_time =
        std::chrono::duration<float>(end_time - start_time).count();
    return {expanded_nodes, -1, elapsed_time,
            expanded_nodes != 0 ? (float)heuristic_total_value / expanded_nodes
                                : 0,
            heuristic_root_value};
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        cerr << "Uso: -<algoritmo> <estado1>,<estado2>,...\n";
        return 1;
    }

    string algorithm = argv[1];
    if (!is_valid_algorithm(algorithm)) {
        cerr << "Erro: Algoritmo invalido.\n";
        return 1;
    }

    vector<vector<int>> states_2d;

    if (argc >= 3) {
        if (!extract_digits(argc, argv, states_2d)) {
            return 1;
        }
    } else {
        if (!extract_digits_from_stream(cin, states_2d)) {
            return 1;
        }
    }

    cout << "Algoritmo: " << algorithm << "\n";
    cout << "Estados:\n";
    for (const auto &state : states_2d) {
        for (const int &num : state) {
            cout << num << " ";
        }
        cout << "\n";
    }

    std::function<output(long long int, int)> selected_algorithm;
    if (algorithm == "-bfs") {
        selected_algorithm = bfs;
    } else if (algorithm == "-gbfs") {
        selected_algorithm = gbfs;
    } else if (algorithm == "-astar") {
        selected_algorithm = astar;
    } else if (algorithm == "-idfs") {
        selected_algorithm = idfs;
    } else if (algorithm == "-idastar") {
        selected_algorithm = idastar;
    } else {
        std::cout << "O algoritmo selecionado ainda não foi implementado.\n";
        return 0;
    }

    ofstream csv_file("results/results_" + algorithm.substr(1) + "_" +
                      std::to_string(state_size - 1) + ".csv");
    if (!csv_file.is_open()) {
        cerr << "Failed to open file.\n";
        return 1;
    }
    csv_file << "expanded_nodes,optimal_solution_length,time,heuristic_avg_"
                "value,heuristic_root_value\n";

    int puzzle_size = states_2d[0].size();
    int n = static_cast<int>(sqrt(puzzle_size));
    for (const auto &state : states_2d) {
        long long int encoded_state = encode_state(state);
        output result = selected_algorithm(encoded_state, n);

        cout << result.expanded_nodes << "," << result.optimal_solution_length
             << "," << result.time << "," << result.heuristic_avg_value << ","
             << result.heuristic_root_value << "\n";

        csv_file << result.expanded_nodes << ","
                 << result.optimal_solution_length << "," << result.time << ","
                 << result.heuristic_avg_value << ","
                 << result.heuristic_root_value << "\n";
    }
    csv_file.close();
    return 0;
}
