# SlackBot
This package provies a class wrapper for the Slack API calls to enable automated interaction with Slack.

See Slack for specific details on how to create a bot for your channel: https://slack.com/help/articles/115005265703-Create-a-bot-for-your-workspace

## Example of basic usage
Basic example of how to use the SlackBot class to message a given channel and upload a file.
This presumes you have create a bot using instructions from Slack and have generated the required slack token for your app.

### Code Example
This example presumes you have copied the slack_token into the text file slack_token.txt in the same directory as this simple script.

```python
from cares_lib.slack_bot.SlackBot import SlackBot

with open('slack_token.txt') as file:
    slack_token = file.read()

slack_bot = SlackBot(slack_token=slack_token)

slack_bot.post_message(channel="#cares-chat-bot", message="Hello <@U010KNR38TZ>")
slack_bot.upload_file(channel="#cares-chat-bot", message="Here is the file", filepath="test/", filename="test.txt")
```