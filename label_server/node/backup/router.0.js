function route(handlers, pathname, response, postData) {
  console.log("About to route a request for " + pathname);
  // 檢查 pathname 是否有對應的 request handlers
  if (typeof handlers[pathname] == "function") {
    handlers[pathname](response, postData);
  } else {
    console.log("No request handler for this pathname: '" + pathname + "'");
    response.writeHead(404, {"Content-Type": "text/plain"});
    response.write("404 Not found");
    response.end();
  }
}

exports.route = route;
