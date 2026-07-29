#pragma once

#include <cstddef>
#include <string_view>

class JsonSyntaxParser
{
public:
    explicit JsonSyntaxParser(std::string_view text) : text_(text) {}

    [[nodiscard]] bool valid()
    {
        skipWhitespace();
        const bool parsed = parseValue();
        skipWhitespace();
        return parsed && position_ == text_.size();
    }

private:
    void skipWhitespace()
    {
        while (position_ < text_.size()
               && (text_[position_] == ' ' || text_[position_] == '\n'
                   || text_[position_] == '\r' || text_[position_] == '\t'))
        {
            ++position_;
        }
    }

    [[nodiscard]] bool consume(char expected)
    {
        skipWhitespace();
        if (position_ >= text_.size() || text_[position_] != expected)
        {
            return false;
        }
        ++position_;
        return true;
    }

    [[nodiscard]] bool parseValue()
    {
        skipWhitespace();
        if (position_ >= text_.size()) { return false; }
        if (text_[position_] == '{') { return parseObject(); }
        if (text_[position_] == '[') { return parseArray(); }
        if (text_[position_] == '"') { return parseString(); }
        if (text_.substr(position_, 4) == "true" || text_.substr(position_, 4) == "null")
        {
            position_ += 4U;
            return true;
        }
        if (text_.substr(position_, 5) == "false")
        {
            position_ += 5U;
            return true;
        }
        return parseNumber();
    }

    [[nodiscard]] bool parseString()
    {
        if (!consume('"')) { return false; }
        while (position_ < text_.size())
        {
            const char character = text_[position_++];
            if (character == '"') { return true; }
            if (character == '\\')
            {
                if (position_ >= text_.size()) { return false; }
                ++position_;
            }
            else if (static_cast<unsigned char>(character) < 0x20U)
            {
                return false;
            }
        }
        return false;
    }

    [[nodiscard]] bool parseNumber()
    {
        const std::size_t begin = position_;
        if (position_ < text_.size() && text_[position_] == '-') { ++position_; }
        while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
        {
            ++position_;
        }
        if (position_ < text_.size() && text_[position_] == '.')
        {
            ++position_;
            while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
            {
                ++position_;
            }
        }
        if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E'))
        {
            ++position_;
            if (position_ < text_.size() && (text_[position_] == '+' || text_[position_] == '-'))
            {
                ++position_;
            }
            while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
            {
                ++position_;
            }
        }
        return position_ > begin;
    }

    [[nodiscard]] bool parseObject()
    {
        if (!consume('{')) { return false; }
        skipWhitespace();
        if (consume('}')) { return true; }
        do
        {
            if (!parseString() || !consume(':') || !parseValue()) { return false; }
            skipWhitespace();
            if (consume('}')) { return true; }
        } while (consume(','));
        return false;
    }

    [[nodiscard]] bool parseArray()
    {
        if (!consume('[')) { return false; }
        skipWhitespace();
        if (consume(']')) { return true; }
        do
        {
            if (!parseValue()) { return false; }
            skipWhitespace();
            if (consume(']')) { return true; }
        } while (consume(','));
        return false;
    }

    std::string_view text_;
    std::size_t position_{};
};
