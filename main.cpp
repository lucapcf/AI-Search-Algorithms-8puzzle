#include <iostream>
#include <sstream>
#include <vector>
#include <set>
#include <unordered_set>
#include <algorithm>

bool is_valid_algorithm(const std::string &alg)
{
    static const std::unordered_set<std::string> valid_algorithms = {"-bfs", "-idfs", "-astar", "-idastar", "-gbfs"};
    return valid_algorithms.count(alg) > 0;
}

bool is_valid_state_size(const std::vector<int> &state, std::vector<int> &expected_sizes)
{
    for (int n : expected_sizes)
    {
        if (state.size() == static_cast<std::size_t>(n))
        {
            expected_sizes = {n};
            return true;
        }
    }
    return false;
}

bool is_valid_puzzle_state(const std::vector<int> &state, int max_value)
{
    std::unordered_set<int> seen_numbers;

    for (int num : state)
    {
        if (num < 0 || num > max_value)
        {
            std::cerr << "Erro: Numero fora do intervalo permitido: " << num << std::endl;
            return false;
        }

        if (seen_numbers.find(num) != seen_numbers.end())
        {
            std::cerr << "Erro: Numero repetido: " << num << std::endl;
            return false;
        }

        seen_numbers.insert(num);
    }

    return true;
}

bool extract_digits(int argc, char *argv[], std::vector<std::vector<int>> &states_2d)
{
    std::vector<int> current_state;
    std::vector<int> expected_sizes = {9, 16};

    for (int i = 2; i < argc; ++i)
    {
        std::string arg = argv[i];
        std::string current_number = "";

        for (char c : arg)
        {
            if (isdigit(c))
            {
                current_number += c;
            }
            else if (c == ',')
            {
                if (!current_number.empty())
                {
                    current_state.push_back(std::stoi(current_number));
                    current_number.clear();
                }

                if (!is_valid_state_size(current_state, expected_sizes))
                {
                    std::cerr << "Erro: Estado invalido. Os estados devem ter 9 ou 16 digitos.\n";
                    return false;
                }

                if (!is_valid_puzzle_state(current_state, expected_sizes[0] - 1))
                {
                    return false;
                }

                states_2d.push_back(current_state);
                current_state.clear();
            }
            else
            {
                std::cerr << "Erro: Caractere invalido: " << c << std::endl;
                return false;
            }
        }

        if (!current_number.empty())
        {
            current_state.push_back(std::stoi(current_number));
        }
    }

    if (!current_state.empty())
    {
        if (!is_valid_state_size(current_state, expected_sizes))
        {
            std::cerr << "Erro: Estado invalido. Os estados devem ter 9 ou 16 digitos.\n";
            return false;
        }

        if (!is_valid_puzzle_state(current_state, expected_sizes[0] - 1))
        {
            return false;
        }

        states_2d.push_back(current_state);
    }

    return true;
}

int main(int argc, char *argv[])
{
    if (argc < 10)
    {
        std::cerr << "Uso: -<algoritmo> <estado1>,<estado2>,...\n";
        return 1;
    }

    std::string algorithm = argv[1];
    if (!is_valid_algorithm(algorithm))
    {
        std::cerr << "Erro: Algoritmo invalido.\n";
        return 1;
    }

    std::vector<std::vector<int>> states_2d;

    if (!extract_digits(argc, argv, states_2d))
    {
        return 1;
    }

    std::cout << "Algoritmo: " << algorithm << "\n";
    std::cout << "Estados:\n";
    for (const auto &state : states_2d)
    {
        for (const int &num : state)
        {
            std::cout << num << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
