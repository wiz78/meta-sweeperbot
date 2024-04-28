
#include "websock_chat.h"

std::set<ChatSocketHandler*> gChatClients;

void ChatSocketHandler::onConnect() {
    puts("Connect!");

    mSessionToken = parseSessionCookie((*mRequest)["Cookie"]);
    
    if (checkSession()) {
        mUser = &gUsers[gUserSessions[mSessionToken]];
        gChatClients.insert(this);
    }
}

void ChatSocketHandler::onTextMessage(const std::string& message) {
    if (!checkSession()) return;

    std::time_t now = std::time(0);

    std::string e;
    auto j = nlohmann::json::parse(message, e);

    if (!e.empty()) {
        sendJson(nlohmann::json {
            { "type", "error" },
            { "error", e },
            { "time", (double) now }
        });

        return;
    }

    if (!j["message"].isString()) {
        sendJson(nlohmann::json {
            { "type", "error" },
            { "error", "invalid message" },
            { "time", (double) now }
        });

        return;
    }

    std::string _message = j["message"].toString();

    if (_message.empty()) {
        sendJson(nlohmann::json {
            { "type", "error" },
            { "error", "empty message" },
            { "time", (double) now }
        });

        return;
    }

    std::cout << " [" << mUser->displayName << "] -> " << _message << std::endl;

    sendJson(nlohmann::json {
        { "type", "text_message" },
        { "sender", nlohmann::json {
            { "id", "$self" },
            { "name", mUser->displayName }
        } },
        { "content", _message },
        { "time", (double) now }
    });

    auto toSend = nlohmann::json {
        { "type", "text_message" },
        { "sender", nlohmann::json {
            { "id", (double) mUser->id },
            { "name", mUser->displayName }
        } },
        { "content", _message },
        { "time", (double) now }
    };

    for (auto client : gChatClients)
        if (client != this)
            client->sendJson(toSend);
}

void ChatSocketHandler::onBinaryMessage(const uint8_t* message, const size_t len) {
    puts("Binary message!");
}

void ChatSocketHandler::onDisconnect() {
    puts("Disconnect!");
    gChatClients.erase(this);
}

bool ChatSocketHandler::checkSession() {
    auto sess = gUserSessions.find(mSessionToken);

    if (sess == gUserSessions.end()) {
        sendDisconnect();
        return false;
    }

    return true;
}
