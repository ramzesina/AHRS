#include "libDM_stream_logger.hpp"

// ##################################################################### //
// ############################# Json tools ############################ //
// ##################################################################### //

bool streamLogger::jsonTools::deserializeJsonCharBuffer()
{
    mJsonDoc.clear();

    // Parse the JSON input
    DeserializationError ret = deserializeJson(mJsonDoc, mJsonCharBuffer);
    // Parse succeeded?
    if (ret != DeserializationError::Ok)
    {
        mStreamLoggerObj->printlnUlog(true,
                                      true,
                                      mClassName + ": deserializeJson() failed with error: "
                                              + String(ret.c_str()) + " " + String(ret.f_str()));
        return false; // NOLINT(readability-simplify-boolean-expr)
    }

    if (mJsonDoc.is<JsonArray>())
        mStreamLoggerObj->printlnUlog(true,
                                      true,
                                      mClassName + ": deserialization success, json size: "
                                              + String(mJsonDoc.memoryUsage()) + "/"
                                              + String(mJsonDoc.capacity())
                                              + " bytes, object type = array");
    else
        mStreamLoggerObj->printlnUlog(true,
                                      true,
                                      mClassName + ": deserialization success, json size: "
                                              + String(mJsonDoc.memoryUsage()) + "/"
                                              + String(mJsonDoc.capacity())
                                              + " bytes, object type = obj");

    return true;
}

bool streamLogger::jsonTools::updateJsonFromStringVector(const std::vector<String>& buffer)
{
    if (buffer.empty())
        return false;

    size_t pos = 0;
    for (size_t i = 0; i < buffer.size(); i++)
    {
        size_t len = buffer[i].length();
        if (pos + len < JSON_FILE_SIZE / 2)
        {
            strncpy(mJsonCharBuffer + pos, buffer[i].c_str(), len);
            pos += len;
        }
        else
        {
            mStreamLoggerObj->errorsHandler(true,
                                            "updateJsonFromStringVector() failed: buffer too big");
            return false;
        }
    }

    // Add null terminator
    mJsonCharBuffer[pos] = static_cast<char>(0);

    return deserializeJsonCharBuffer();
}

void streamLogger::jsonTools::printJsonDoc()
{
    printJsonDoc(mJsonDoc);
}

void streamLogger::jsonTools::printJsonDoc(DynamicJsonDocument& jsonDoc)
{
    String jsonDocPretty = "Json size = " + String(jsonDoc.memoryUsage()) + "/"
                           + String(jsonDoc.capacity()) + " bytes, Json content:\n";
    serializeJsonPretty(jsonDoc, jsonDocPretty);
    mStreamLoggerObj->printlnUlog(true, true, jsonDocPretty);
}

bool streamLogger::jsonTools::writeJsonDoc(streamLogger::INTERFACE interface, String filePath)
{
    return writeJsonDoc(interface, filePath, mJsonDoc);
}
