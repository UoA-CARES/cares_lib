# SlackBot
This package provies a class wrapper for the Slack API calls to enable automated interaction with Slack.

# Example of basic usage
Basic example of how to use the SlackBot class to message a given channel and upload a file.
Other features to be added in the future.

```python
from cares_lib.slack_bot.SlackBot import SlackBot

with open('slack_token.txt') as file:
    slack_token = file.read()

slack_bot = SlackBot(slack_token=slack_token)

slack_bot.post_message(channel="#cares-chat-bot", message="Hello <@U010KNR38TZ>")
slack_bot.upload_file(channel="#cares-chat-bot", message="Here is the file", filepath="test/", filename="test.txt")
```