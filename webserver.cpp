/*
 * Small web-server to serve powerscan files and data streams
 */
extern "C" {
#include "powerscan.h"
}

#include <chrono>
#include <cstdio>
#include <thread>
#include <httplib.h>

using namespace httplib;

static Server*	server;

static void run_webserver(ProgramConfiguration* pc);

void
setup_webserver(ProgramConfiguration* pc)
{
	if (pc->web_port == 0)
		return;

	(void) new std::thread([pc] {
		printf("starting thread\n");
		run_webserver(pc);
		printf("thread finished\n");
	});
}

void
stop_webserver()
{
	if (server)
	{
		server->stop();
		server = 0;
	}
}

static void
run_webserver(ProgramConfiguration* pc)
{
	server = new Server;

	if (!server->is_valid()) {
		printf("Can't start web server on localhost:%d\n", pc->web_port);
		return;
	}

	server->set_keep_alive_max_count(20); // Default is 5
	server->set_keep_alive_timeout(60);  // Default is 5
	server->set_logger([](const Request &req, const Response &res) {
		(void)(res);
		printf("%s %s\n", req.method.c_str(), req.path.c_str());
	});

	server->set_error_handler([](const Request & /*req*/, Response &res) {
		const char *fmt = "<p>Error Status: <span style='color:red;'>%d</span></p>";
		char buf[BUFSIZ];
		snprintf(buf, sizeof(buf), fmt, res.status);
		res.set_content(buf, "text/html");
	});

	printf("Starting web server on localhost:%d\n", pc->web_port);

	server->Get("/", [=](const Request & /*req*/, Response &res) {
	  res.set_redirect("/index.html");
	});

	// REVISIT: Need to locate the public directory
	if (!server->set_mount_point("/", "./public"))
	{
		printf("Can't serve public files");
		return;
	}

	// REVISIT: Example of serving a chunked response:
	server->Get("/chunked", [&](const Request& req, Response& res) {
		(void)(req);
		printf("chunk starting...\n");
		res.set_chunked_content_provider(
			"text/plain",
			[](size_t offset, DataSink &sink) {
			  	(void)(offset);
			  	sink.write("123\n\n", 5);
			  	sink.write("345\n\n", 5);
			  	std::this_thread::sleep_for(std::chrono::seconds(2));
			  	sink.write("789\n\n", 5);
			  	sink.done(); // No more data
			  	printf("chunk done...\n");
			  	return true; // return 'false' if you want to cancel the process.
			}
		);
	});

	server->listen("localhost", pc->web_port);

	printf("webserver done\n");
}
