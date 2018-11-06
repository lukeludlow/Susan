#!/bin/bash
echo "Cache-Control: no-cache, no-store, must-revalidate";
echo "Pragma: no-cache";
echo "Expires: 0";
echo "Content-Type: application/json;charset=UTF-8"
echo ""
echo "{ \"version\": \"1.0\", \"response\": { \"outputSpeech\": {\"type\": \"PlainText\", \"text\": \"Got it. Yes, your command was received.\"} }}"
