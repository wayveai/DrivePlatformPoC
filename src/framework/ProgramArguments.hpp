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
// SPDX-FileCopyrightText: Copyright (c) 2014-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef SAMPLES_COMMON_PROGRAMARGUMENTS_HPP_
#define SAMPLES_COMMON_PROGRAMARGUMENTS_HPP_

#include <map>
#include <string>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <stdexcept>
#include <set>

// EXAMPLE USECASE:
//
//    ProgramArguments arguments(
//    { ProgramArguments::Option_t("optional_option", "with default value"),
//    ProgramArguments::Option_t("required_option")
//    }
//    );
//
//    if (!arguments.parse(argc, argv)) exit(0); // Exit if not all require arguments are provided
//    printf("Program Arguments:\n%s\n", arguments.printList());
//
class ProgramArguments
{
public:
    struct Option_t
    {
        Option_t(const char* option_, const char* default_value_, const char* help_)
        {
            option        = option_;
            default_value = default_value_;
            help          = help_;
            required      = false;
            array         = false;
            parsed        = false;
            value         = default_value;
        }
        Option_t(const char* option_, std::nullptr_t, const char* help_)
        {
            option        = option_;
            default_value = "";
            help          = help_;
            required      = true;
            array         = false;
            parsed        = false;
            value         = "";
        }

        Option_t(const char* option_, const char* default_value_)
        {
            option        = option_;
            default_value = default_value_;
            required      = false;
            array         = false;
            parsed        = false;
            value         = default_value;
        }

        Option_t(const char* option_)
        {
            option        = option_;
            default_value = "";
            required      = true;
            array         = false;
            parsed        = false;
        }

        Option_t(const char* option_, const bool required_)
        {
            option        = option_;
            default_value = "";
            required      = required_;
            array         = false;
            parsed        = false;
        }

        Option_t(const char* option_, const std::vector<std::string>& default_values_, const char* help_)
        {
            option         = option_;
            default_values = default_values_;
            help           = help_;
            required       = false;
            array          = true;
            parsed         = false;
            values         = default_values;
        }

        Option_t(const char* option_, const std::vector<std::string>& default_values_)
        {
            option         = option_;
            default_values = default_values_;
            required       = false;
            array          = true;
            parsed         = false;
            values         = default_values;
        }

        std::string option;
        std::string default_value;
        std::vector<std::string> default_values;
        std::string help;
        bool required;
        bool array;

        bool parsed;
        std::string value;
        std::vector<std::string> values;
    };

public:
    ProgramArguments(bool addDefaults = true);
    ProgramArguments(int32_t argc, const char** argv, const std::vector<Option_t>& options, const char* description = nullptr);
    ProgramArguments(const std::vector<Option_t>& options, bool addDefaults = true);
    virtual ~ProgramArguments();

    bool parse(const int argc, const char** argv);

    virtual bool has(const char* name) const;
    bool wasSpecified(const char* name) const;
    bool isRequired(const char* name) const;
    virtual bool enabled(const char* name) const;
    void addOption(const Option_t& newOption);
    void addOptions(const std::vector<Option_t>& newOptions);
    void clearOption(const char* name);
    void clearOptions();

    virtual const std::string& get(const char* name) const;
    void set(const char* option, const char* value);
    void setDefault(const char* option, const char* value);

    virtual const std::vector<std::string>& getArray(const char* name) const;
    void set(const char* option, const std::vector<std::string>& values);
    void setDefault(const char* option, const std::vector<std::string>& values);

    // Will be shown before the help text
    void setDescription(const char* description);

    /// Displays a message with info about the possible parameters
    virtual void printHelp() const;

    /// Returns a string with info about the parameter values
    std::string printList() const;

    /// Returns all the parsed parameters as a single string
    std::string parameterString() const;

    std::set<std::string> getOptionNames() const;

    // Map the current argument value to a fixed number of value variants, enabling code like
    //  getEnum("fast-acceptance", {"default", "enabled", "disabled"},
    //          {DW_CALIBRATION_FAST_ACCEPTANCE_DEFAULT,
    //           DW_CALIBRATION_FAST_ACCEPTANCE_ENABLED,
    //           DW_CALIBRATION_FAST_ACCEPTANCE_DISABLED})
    template <class T>
    T getEnum(const char* const name,
              std::unordered_map<std::string, T> const& variants) const;

    // Note: separated into two initializer lists so T can be inferred
    template <class T>
    T getEnum(const char* const name,
              std::initializer_list<std::string> const& variantNames,
              std::initializer_list<T> const& optionValues) const;

protected:
    std::string m_empty                   = {};
    std::vector<std::string> m_emptyArray = {};
    std::map<std::string, Option_t> arguments;
    std::string m_description = {};
};

/////////////////////////////////////////////////
// Implementation
template <class T>
T ProgramArguments::getEnum(const char* const name,
                            std::unordered_map<std::string, T> const& variants) const
{
    auto const s  = get(name);
    auto const it = variants.find(s);
    if (it != variants.end())
    {
        return it->second;
    }
    else
    {
        std::stringstream ss;
        ss << "ProgramArguments::getEnum: Invalid argument for " << name << ". Variants: ";
        for (auto const& v : variants)
        {
            if (&v != &*variants.begin())
            {
                ss << ", ";
            }
            ss << v.first;
        }
        ss << ". Received: " + s;
        throw std::runtime_error(ss.str());
    }
}

template <class T>
T ProgramArguments::getEnum(const char* const name,
                            std::initializer_list<std::string> const& variantNames,
                            std::initializer_list<T> const& variantValues) const
{
    if (variantNames.size() != variantValues.size())
    {
        throw std::runtime_error(std::string{"ProgramArguments::getEnum: Size of initializer lists don't match for option "} + name);
    }
    std::unordered_map<std::string, T> variants;
    auto itName  = variantNames.begin();
    auto itValue = variantValues.begin();
    for (; itName != variantNames.end(); ++itName, ++itValue)
    {
        variants.insert({*itName, *itValue});
    }

    return getEnum(name, variants);
}

#endif // SAMPLES_COMMON_PROGRAMARGUMENTS_HPP_
