import { isIP } from "node:net";

export const invalidSourceUrlMessage = "Enter a valid URL.";
export const blockedSourceUrlMessage = "Enter a public https://, s3://, or gs:// link.";

const internalHostSuffixes = [".localhost", ".local", ".internal"];

// A web link must name a public host. An IP address, a single-label name, or
// an internal name can point a downloader at a private service, for example
// the cloud metadata server.
function isPublicHostname(hostname: string) {
  const host = hostname.toLowerCase().replace(/\.$/, "");

  if (!host.includes(".")) {
    return false;
  }

  // URL keeps the brackets on an IPv6 host, and it rewrites every IPv4
  // spelling (hex, octal, one integer) to the dotted form.
  if (host.startsWith("[") || isIP(host) !== 0) {
    return false;
  }

  return !internalHostSuffixes.some((suffix) => host.endsWith(suffix));
}

export function parseSourceUrl(value: unknown) {
  if (typeof value !== "string") {
    return undefined;
  }

  const sourceUrl = value.trim().slice(0, 2_048);
  if (!sourceUrl) {
    return undefined;
  }

  let parsedUrl: URL;
  try {
    parsedUrl = new URL(sourceUrl);
  } catch {
    throw new Error(invalidSourceUrlMessage);
  }

  if (parsedUrl.protocol === "s3:" || parsedUrl.protocol === "gs:") {
    return parsedUrl.toString();
  }

  if (
    parsedUrl.protocol !== "https:"
    || parsedUrl.username
    || parsedUrl.password
    || !isPublicHostname(parsedUrl.hostname)
  ) {
    throw new Error(blockedSourceUrlMessage);
  }

  return parsedUrl.toString();
}
