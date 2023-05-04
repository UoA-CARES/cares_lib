import logging
from functools import wraps

import slack
from slack.errors import SlackApiError


def exception_handler(error_message):
    def decorator(function):
        @wraps(function)
        def wrapper(self, *args, **kwargs):
            try:
                return function(self, *args, **kwargs)
            except SlackApiError as error:
                logging.error(f"Error: {error} with error message: {error_message}")
        return wrapper
    return decorator

class SlackBot:
  def __init__(self, slack_token):
    self.client = slack.WebClient(token=slack_token)
    
  @exception_handler("Failed to post message")
  def post_message(self, channel, message):
    self.client.chat_postMessage(channel=channel,text=message)

  @exception_handler("Failed to get message")
  def get_message(self, channel):
    conversation_id = self.get_id(channel)
    result = self.client.conversations_history(
        channel=conversation_id,
        inclusive=True,
        limit=1
    )

    message = result["messages"][0]
    return message["text"]

  @exception_handler("Failed to get ids")
  def get_id(self, channel_name):
    conversation_id = None
    for result in self.client.conversations_list():
      if conversation_id is not None:
          break
      for channel in result["channels"]:
        if channel["name"] == channel_name:
          conversation_id = channel["id"]
          return conversation_id
  
  @exception_handler("Failed to upload file")
  def upload_file(self, channel, message, filepath, filename):
    self.client.files_upload(channels=channel, title=filename, file=filepath+filename, initial_comment=message)