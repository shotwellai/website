import { Resolver } from "node:dns/promises";

// Mail hosts of disposable mailbox services. A disposable service adds new
// domains all the time, but each new domain points its MX record at the same
// mail host. Each entry was confirmed with an MX lookup on 2026-11-15.
const disposableMailHosts = [
  "mail.tm",
  "mailinator.com",
  "yopmail.com",
  "guerrillamail.com",
  "maildrop.cc",
  "getnada.com",
  "dropmail.me",
  "minuteinbox.com",
  "1secmail.com",
  "trashmail.com",
  "tempmail.net",
  "mailnesia.com",
  "dispostable.com",
  "moakt.com",
  "fex.plus"
];

export type EmailDomainStatus = "ok" | "disposable" | "missing";

const resolver = new Resolver({ timeout: 2_000, tries: 2 });

function isDisposableHost(host: string) {
  const name = host.toLowerCase().replace(/\.$/, "");
  return disposableMailHosts.some((entry) => name === entry || name.endsWith(`.${entry}`));
}

export async function checkEmailDomain(email: string): Promise<EmailDomainStatus> {
  const domain = email.slice(email.lastIndexOf("@") + 1);

  if (isDisposableHost(domain)) {
    return "disposable";
  }

  try {
    const records = await resolver.resolveMx(domain);
    return records.some((record) => isDisposableHost(record.exchange)) ? "disposable" : "ok";
  } catch (error) {
    // ENOTFOUND: the domain does not exist, so the link cannot arrive.
    // Any other DNS failure must not lock out a real person.
    return (error as NodeJS.ErrnoException).code === "ENOTFOUND" ? "missing" : "ok";
  }
}
