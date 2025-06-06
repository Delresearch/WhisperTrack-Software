
using Sockets

# Define a function to send the file over an existing socket
function send_file(sock::TCPSocket, filename::String)
    open(filename, "r") do f

        chunk = read(f, 16*1024)
        write(sock, chunk)

    end
    flush(sock)  # Ensure all data is pushed to the socket
end

# Check that we have at least one argument for the filename
if length(ARGS) < 1
    println("Usage: $(basename(PROGRAM_FILE)) <filename>")
    exit(1)
end

filename = ARGS[1]

# Create and use the TCP socket
sock = connect("127.0.0.1", 19021)
send_file(sock, filename)
close(sock)