/////////////////////////////////////////////////////////////////////////////////////////
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
// expressly authorized by NVIDIA. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA CORPORATION & AFFILIATES.
//
// SPDX-FileCopyrightText: Copyright (c) 2018-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef STM_TESTS_DETAILS_ARGPARSE_H
#define STM_TESTS_DETAILS_ARGPARSE_H

#include <stdexcept>
#include <functional>
#include <unordered_map>
#include <vector>
#include <sstream>

namespace stm
{
namespace manager
{
class ArgParse final
{
    // Note: this class is rudimentary and untested.
public:
    using ActionLambda = std::function<void(std::string)>;

    class Actions
    {
    public:
        static ActionLambda enableFlag(bool& flag)
        {
            return [&flag](std::string) { flag = true; };
        }

        static ActionLambda copyToString(std::string& destination)
        {
            return [&destination](std::string value) { destination = value; };
        }
        static ActionLambda copyToInt(int32_t& intValue)
        {
            return [&intValue](std::string value) { intValue = std::stoi(value); };
        }
        static ActionLambda copyToInt(uint64_t& intValue)
        {
            return [&intValue](std::string value) { intValue = std::stoi(value); };
        }
    };

private:
    struct OptionalArgument
    {
        ActionLambda action;
        std::string shortIdentifier;
        std::string description;
        bool flag;
    };

    struct MandatoryArgument
    {
        std::string id;
        ActionLambda action;
        std::string description;
    };

    bool m_markExit;
    bool m_error;
    std::string m_description;
    std::unordered_map<std::string, std::shared_ptr<OptionalArgument>> m_optionalArguments;
    std::vector<std::shared_ptr<MandatoryArgument>> m_mandatoryArguments;

public:
    ArgParse(std::string description)
        : m_markExit(false)
        , m_error(false)
        , m_description(std::move(description))
    {
        // Note that the below will break if the object is moved between construction and parsing.
        // This is why we delete those constructors.
        static auto helpAction = [this](std::string) { std::cout << usage() << std::endl << std::flush; m_markExit = true; };
        addOptionalArgument("--help", helpAction, "Print this message and have the program exit.", "-h");
    }

    ArgParse(const ArgParse&) = delete;            // Copy constructor.
    ArgParse& operator=(const ArgParse&) = delete; // Copy assignment.

    ArgParse(ArgParse&& other) = delete;      // Move constructor.
    ArgParse& operator=(ArgParse&&) = delete; // Move Assignment

    ~ArgParse() = default;

    void addMandatoryArgument(std::string id, ActionLambda action, std::string description = "")
    {
        // Note: no error checking performed on id.

        m_mandatoryArguments.push_back(std::shared_ptr<MandatoryArgument>(new MandatoryArgument{
            std::move(id),
            std::move(action),
            std::move(description)}));
    }

    void addOptionalArgument(std::string id, ActionLambda action, std::string description = "", std::string shortIdentifier = "", bool flag = false)
    {
        // Note: no error checking performed on id.

        auto optionalArgument = std::shared_ptr<OptionalArgument>(new OptionalArgument{
            std::move(action),
            shortIdentifier,
            std::move(description),
            flag});
        m_optionalArguments[std::move(id)] = optionalArgument;

        if (!shortIdentifier.empty())
            m_optionalArguments[std::move(shortIdentifier)] = optionalArgument;
    }

    std::string usage()
    {
        std::stringstream usage;
        usage << m_description << std::endl
              << std::endl;

        usage << "The following arguments are MANDATORY:" << std::endl;
        for (auto& arg : m_mandatoryArguments)
        {
            usage << "\t" << arg->id;
            if (!arg->description.empty())
                usage << ": " << arg->description;
            usage << std::endl;
        }

        usage << "The following arguments are OPTIONAL:" << std::endl;
        for (auto& argPair : m_optionalArguments)
        {
            usage << "\t" << argPair.first;
            if (!argPair.second->shortIdentifier.empty())
                usage << " [" << argPair.second->shortIdentifier << "]";
            if (!argPair.second->description.empty())
                usage << ": " << argPair.second->description;
            usage << std::endl;
        }

        return usage.str();
    }

    void parse(int argc, const char** argv)
    {
        try
        {
            size_t mandatoryArgumentIndex = 0;
            for (int i = 1; i < argc; i++)
            {
                std::string argument = argv[i];

                // Check to see if this argument is optional. these can be interleaved between mandatory arguments.
                auto optionalFound = m_optionalArguments.find(argument);
                if (optionalFound != m_optionalArguments.end())
                {
                    if (optionalFound->second->flag)
                        optionalFound->second->action(argument);
                    else
                    {
                        // The optional argument is not a flag; we need to consume the subsequent command line argument.
                        if (i >= argc)
                            throw std::invalid_argument("Too few arguments provided.");

                        i++;
                        argument = argv[i];

                        optionalFound->second->action(argument);
                    }
                    exitIfMarked(); // Check exit marking after every action.
                    continue;
                }

                if (mandatoryArgumentIndex == m_mandatoryArguments.size())
                    // Assume this is a mandatory argument.
                    throw std::invalid_argument("One or more arguments are invalid.");

                m_mandatoryArguments[mandatoryArgumentIndex]->action(argument);
                exitIfMarked(); // Check exit marking after every action.
                mandatoryArgumentIndex++;
            }

            // Check that all mandatory arguments are used.
            if (mandatoryArgumentIndex != m_mandatoryArguments.size())
                throw std::invalid_argument("Too few mandatory arguments provided.");
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            std::cerr << usage() << std::endl;
            std::cerr << std::flush;

            m_markExit = true;
            m_error    = true;
        }

        exitIfMarked();
    }

private:
    inline void exitIfMarked()
    {
        if (m_markExit)
            exit(m_error ? EXIT_FAILURE : EXIT_SUCCESS);
    }
};
}
}

#endif //STM_TESTS_DETAILS_ARGPARSE_H
