#include <cstdarg>
#include <string>

#include "gtest/gtest.h"
#include "object_dictionary.hh"

static uint16_t Format(ObjectDictionary::LogMessage& msg, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    uint16_t n = ObjectDictionary::FormatLogMessage(msg, tag, format, args);
    va_end(args);
    return n;
}

TEST(LogMessage, FormatsTagAndMessage) {
    ObjectDictionary::LogMessage msg;
    EXPECT_EQ(Format(msg, "Tag", "value=%d", 42), strlen("[Tag] value=42"));
    EXPECT_STREQ(msg.message, "[Tag] value=42");
    EXPECT_EQ(Format(msg, "", "no tag"), strlen("no tag"));
    EXPECT_STREQ(msg.message, "no tag");
}

// num_chars must be clamped to kLogMessageMaxNumChars for long messages.
TEST(LogMessage, ClampsLongMessages) {
    ObjectDictionary::LogMessage msg;
    std::string long_text(600, 'x');
    uint16_t n = Format(msg, "SomeTag", "%s", long_text.c_str());
    EXPECT_EQ(n, ObjectDictionary::kLogMessageMaxNumChars);
    EXPECT_EQ(msg.num_chars, ObjectDictionary::kLogMessageMaxNumChars);
    EXPECT_EQ(strlen(msg.message), ObjectDictionary::kLogMessageMaxNumChars);
    EXPECT_EQ(msg.message[ObjectDictionary::kLogMessageMaxNumChars], '\0');

    // A tag that alone fills the buffer can't push the message offset past the end either.
    std::string long_tag(ObjectDictionary::kLogMessageTagMaxNumChars + 50, 't');
    n = Format(msg, long_tag.c_str(), "%s", long_text.c_str());
    EXPECT_LE(n, ObjectDictionary::kLogMessageMaxNumChars);
    EXPECT_EQ(strlen(msg.message), n);
}
