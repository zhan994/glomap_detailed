#include "glomap/exe/global_mapper.h"

#include <colmap/util/logging.h>

#include <iostream>

namespace {

typedef std::function<int(int, char**)> command_func_t;

// api: 指令帮助提示
int ShowHelp(
    const std::vector<std::pair<std::string, command_func_t>>& commands) {
  std::cout << "GLOMAP -- Global Structure-from-Motion" << std::endl
            << std::endl;

  std::cout << "Usage:" << std::endl;
  std::cout << "  glomap mapper --database_path DATABASE --output_path MODEL"
            << std::endl;
  std::cout << "  glomap mapper_resume --input_path MODEL_INPUT --output_path "
               "MODEL_OUTPUT"
            << std::endl;

  std::cout << "Available commands:" << std::endl;
  std::cout << "  help" << std::endl;
  for (const auto& command : commands) {
    std::cout << "  " << command.first << std::endl;
  }
  std::cout << std::endl;

  return EXIT_SUCCESS;
}

}  // namespace

// api: 整个工程的主函数
int main(int argc, char** argv) {
  // step: 1 初始化日志
  colmap::InitializeGlog(argv);
  FLAGS_alsologtostderr = true;

  // step: 2 指令
  std::vector<std::pair<std::string, command_func_t>> commands;
  commands.emplace_back("mapper", &glomap::RunMapper); // note: 核心接口
  commands.emplace_back("mapper_resume", &glomap::RunMapperResume);

  // step: 2.1 如果只有glomap，则提示上述emplace_back的所有cmd
  if (argc == 1) {
    return ShowHelp(commands);
  }

  // step: 3 指令解析并运行
  const std::string command = argv[1];
  // step: 3.1 针对argv[1]进行ShowHelp
  if (command == "help" || command == "-h" || command == "--help") {
    return ShowHelp(commands);
  } else {
    command_func_t matched_command_func = nullptr;
    // step: 3.2 若匹配上述加入的cmd，取出对应的func
    for (const auto& command_func : commands) {
      if (command == command_func.first) {
        matched_command_func = command_func.second;
        break;
      }
    }

    // step: 3.3 调用匹配的func
    if (matched_command_func == nullptr) {
      std::cout << "Command " << command << " not recognized. "
                << "To list the available commands, run `colmap help`."
                << std::endl;
      return EXIT_FAILURE;
    } else {
      int command_argc = argc - 1;
      char** command_argv = &argv[1];
      command_argv[0] = argv[0];
      return matched_command_func(command_argc, command_argv);
    }
  }

  return ShowHelp(commands);
}
