#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "sail/boat_data.hpp"

namespace sail::dossier {

// Keeps the file's key order, so items load in the order the dossier lists them.
using Json = nlohmann::ordered_json;

// A JSON node plus where it came from, so every error names the file and the field.
class Node {
public:
    Node(const std::filesystem::path& file, const Json& json, std::string where)
        : file_(&file), json_(&json), where_(std::move(where)) {}

    [[noreturn]] void fail(const std::string& message) const {
        throw BoatDataError(*file_, (where_.empty() ? std::string("top level") : where_) + ": " +
                                        message);
    }

    bool has(const std::string& key) const {
        return json_->is_object() && json_->contains(key);
    }

    Node at(const std::string& key) const {
        if (!json_->is_object()) fail("expected an object");
        const auto it = json_->find(key);
        if (it == json_->end()) fail("missing field '" + key + "'");
        return Node(*file_, *it, where_.empty() ? key : where_ + "." + key);
    }

    double number() const {
        if (!json_->is_number()) fail("expected a number");
        const double v = json_->get<double>();
        if (!std::isfinite(v)) fail("expected a finite number");
        return v;
    }

    bool boolean() const {
        if (!json_->is_boolean()) fail("expected true or false");
        return json_->get<bool>();
    }

    std::string string() const {
        if (!json_->is_string()) fail("expected a string");
        return json_->get<std::string>();
    }

    std::size_t size() const {
        if (!json_->is_array()) fail("expected an array");
        return json_->size();
    }

    Node operator[](std::size_t i) const {
        if (i >= size()) fail("index " + std::to_string(i) + " out of range");
        return Node(*file_, (*json_)[i], where_ + "[" + std::to_string(i) + "]");
    }

    Vec3 vec3() const {
        if (size() != 3) fail("expected 3 numbers");
        return Vec3((*this)[0].number(), (*this)[1].number(), (*this)[2].number());
    }

    // The "value" of a dossier entry {value, unit, source, ...}.
    double value() const { return at("value").number(); }
    double positive_value() const {
        const double v = value();
        if (!(v > 0.0)) at("value").fail("expected a positive number");
        return v;
    }

    const Json& json() const { return *json_; }
    const std::string& where() const { return where_; }

private:
    const std::filesystem::path* file_;
    const Json* json_;
    std::string where_;
};

struct Document {
    std::filesystem::path file;
    Json json;

    Node root() const { return Node(file, json, ""); }
};

inline Document read_document(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw BoatDataError(file, "cannot open file");
    Document doc{file, {}};
    try {
        doc.json = Json::parse(in);
    } catch (const Json::parse_error& e) {
        throw BoatDataError(file, std::string("malformed JSON: ") + e.what());
    }
    if (!doc.json.is_object()) throw BoatDataError(file, "top level: expected an object");
    return doc;
}

}  // namespace sail::dossier
