import { Router, type Request, type Response } from "express";

import { readSessionCookie, setSessionCookie } from "../auth/cookies.js";
import { authStore, type User } from "../auth/store.js";
import { config } from "../config.js";
import { notifyUploadSubmitted } from "../email/notifications.js";
import { appPage, messagePage } from "../http/render.js";
import { createGcsUploadSessions, createResultReadStream, type NewUploadFileInput } from "../uploads/gcs.js";
import { createUploadId, uploadStore } from "../uploads/store.js";

export const appRouter = Router();

function loginUrl() {
  const url = new URL("/login", config.authBaseUrl);
  url.searchParams.set("return_to", new URL("/auth/complete", config.appBaseUrl).toString());
  return url.toString();
}

async function getAppUser(req: Request, res: Response): Promise<User | null> {
  const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

  if (!session) {
    res.redirect(302, loginUrl());
    return null;
  }

  const user = await authStore.getUser(session.userId);
  if (!user) {
    res.redirect(302, loginUrl());
    return null;
  }

  return user;
}

async function getApiUser(req: Request, res: Response): Promise<User | null> {
  const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

  if (!session) {
    res.status(401).json({ error: "unauthorized" });
    return null;
  }

  const user = await authStore.getUser(session.userId);
  if (!user) {
    res.status(401).json({ error: "unauthorized" });
    return null;
  }

  return user;
}

function cleanFileName(value: unknown) {
  if (typeof value !== "string") {
    return "";
  }

  return value.trim().slice(0, 512);
}

function parseSourceUrl(value: unknown) {
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
    throw new Error("Enter a valid URL.");
  }

  if (!["http:", "https:", "s3:", "gs:"].includes(parsedUrl.protocol)) {
    throw new Error("Enter a valid URL.");
  }

  return parsedUrl.toString();
}

function parseUploadRequest(body: unknown): { prompt: string; files: NewUploadFileInput[]; sourceUrl?: string } {
  if (!body || typeof body !== "object") {
    throw new Error("Invalid upload request.");
  }

  const input = body as { prompt?: unknown; files?: unknown; sourceUrl?: unknown };
  const filesInput = Array.isArray(input.files) ? input.files : [];
  const files = filesInput.map((file) => {
    const item = file && typeof file === "object"
      ? file as { name?: unknown; type?: unknown; size?: unknown }
      : {};
    const originalName = cleanFileName(item.name);

    if (!originalName) {
      throw new Error("Every file needs a name.");
    }

    return {
      originalName,
      contentType: typeof item.type === "string" && item.type.trim() ? item.type.trim().slice(0, 200) : undefined,
      sizeBytes: typeof item.size === "number" && Number.isFinite(item.size) ? item.size : undefined
    };
  });
  const sourceUrl = parseSourceUrl(input.sourceUrl);

  if (files.length === 0 && !sourceUrl) {
    throw new Error("Choose files or provide a link.");
  }

  if (files.length > 0 && sourceUrl) {
    throw new Error("Choose files or provide a link, not both.");
  }

  if (files.length > 100) {
    throw new Error("Upload at most 100 files at a time.");
  }

  return {
    prompt: typeof input.prompt === "string" ? input.prompt.trim().slice(0, 20_000) : "",
    files,
    sourceUrl
  };
}

function parsePromptInput(body: unknown) {
  if (!body || typeof body !== "object") {
    return "";
  }

  const input = body as { prompt?: unknown };
  return typeof input.prompt === "string" ? input.prompt.trim().slice(0, 20_000) : "";
}

function appScript() {
  return `"use strict";

function setUploadMessage(element, message, isError) {
  if (!element) return;
  element.hidden = false;
  element.textContent = message;
  element.classList.toggle("neutral", !isError);
}

async function requestUploadSession(prompt, files, sourceUrl) {
  const response = await fetch("/api/uploads", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      prompt,
      sourceUrl,
      files: files.map((file) => ({
        name: file.name,
        type: file.type || "application/octet-stream",
        size: file.size
      }))
    })
  });

  if (!response.ok) {
    const payload = await response.json().catch(() => ({}));
    throw new Error(payload.error || "Could not start upload.");
  }

  return response.json();
}

async function uploadFileToGcs(session, file) {
  const response = await fetch(session.uploadUrl, {
    method: "PUT",
    headers: { "Content-Type": session.contentType || file.type || "application/octet-stream" },
    body: file
  });

  if (!response.ok) {
    throw new Error("Upload failed for " + file.name + ".");
  }
}

async function markUploadComplete(uploadId) {
  await fetch("/api/uploads/" + encodeURIComponent(uploadId) + "/uploaded", {
    method: "POST"
  });
}

document.addEventListener("DOMContentLoaded", () => {
  const form = document.querySelector("[data-upload-form]");
  if (!form) return;

  const fileInput = form.querySelector("[data-upload-files]");
  const sourceUrlInput = form.querySelector("[data-upload-source-url]");
  const promptInput = form.querySelector("[data-upload-prompt]");
  const submit = form.querySelector("[data-upload-submit]");
  const message = form.querySelector("[data-upload-message]");
  const tabs = Array.from(form.querySelectorAll("[data-upload-tab]"));
  const panels = Array.from(form.querySelectorAll("[data-upload-panel]"));

  function setUploadMode(mode) {
    form.dataset.uploadMode = mode;
    for (const tab of tabs) {
      const isActive = tab.dataset.uploadTab === mode;
      tab.setAttribute("aria-selected", String(isActive));
    }

    for (const panel of panels) {
      panel.hidden = panel.dataset.uploadPanel !== mode;
    }

    if (fileInput) {
      fileInput.disabled = mode !== "files";
      if (mode !== "files") {
        fileInput.value = "";
      }
    }

    if (sourceUrlInput) {
      sourceUrlInput.disabled = mode !== "url";
      if (mode !== "url") {
        sourceUrlInput.value = "";
      }
    }
  }

  for (const tab of tabs) {
    tab.addEventListener("click", () => {
      setUploadMode(tab.dataset.uploadTab || "files");
    });
  }

  setUploadMode("files");

  form.addEventListener("submit", async (event) => {
    event.preventDefault();
    const uploadMode = form.dataset.uploadMode || "files";
    const files = uploadMode === "files" ? Array.from(fileInput.files || []) : [];
    const sourceUrl = uploadMode === "url" ? (sourceUrlInput ? sourceUrlInput.value : "").trim() : "";

    if (uploadMode === "files" && files.length === 0) {
      setUploadMessage(message, "Choose at least one file to upload.", true);
      return;
    }

    if (uploadMode === "url" && !sourceUrl) {
      setUploadMessage(message, "Enter a URL to episodes.", true);
      return;
    }

    if (files.length > 0 && sourceUrl) {
      setUploadMessage(message, "Choose files or provide a link, not both.", true);
      return;
    }

    submit.disabled = true;
    setUploadMessage(message, "Preparing upload...", false);

    try {
      const payload = await requestUploadSession(promptInput.value || "", files, sourceUrl);
      for (let index = 0; index < files.length; index += 1) {
        setUploadMessage(message, "Uploading " + (index + 1) + " of " + files.length + "...", false);
        await uploadFileToGcs(payload.files[index], files[index]);
      }

      if (files.length > 0) {
        await markUploadComplete(payload.upload.id);
      }

      setUploadMessage(message, sourceUrl ? "Link received. Refreshing..." : "Upload received. Refreshing...", false);
      window.location.reload();
    } catch (error) {
      setUploadMessage(message, error instanceof Error ? error.message : "Upload failed.", true);
      submit.disabled = false;
    }
  });
});
`;
}

appRouter.get("/", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const uploads = await uploadStore.listUserUploads(user.id);
    res.type("html").send(appPage(user, uploads));
  } catch (error) {
    next(error);
  }
});

appRouter.get("/app.js", (_req, res) => {
  res.type("application/javascript").send(appScript());
});

appRouter.post("/api/uploads", async (req, res, next) => {
  try {
    const user = await getApiUser(req, res);
    if (!user) {
      return;
    }

    const input = parseUploadRequest(req.body);
    const uploadId = createUploadId();
    const origin = new URL(config.appBaseUrl).origin;
    const sessions = input.files.length > 0
      ? await createGcsUploadSessions({
        userId: user.id,
        uploadId,
        files: input.files,
        origin
      })
      : [];
    const uploadPrefix = [
      config.uploads.uploadPrefix,
      user.id,
      uploadId
    ].filter(Boolean).join("/");

    const upload = await uploadStore.createUpload({
      id: uploadId,
      userId: user.id,
      prompt: input.prompt,
      sourceUrl: input.sourceUrl,
      uploadPrefix,
      files: sessions.map((file) => ({
        id: file.id,
        originalName: file.originalName,
        objectName: file.objectName,
        contentType: file.contentType,
        sizeBytes: file.sizeBytes
      }))
    });

    if (upload.sourceUrl) {
      await notifyUploadSubmitted(user, upload);
    }

    res.json({
      upload: {
        id: upload.id,
        status: upload.status
      },
      files: sessions.map((file) => ({
        id: file.id,
        originalName: file.originalName,
        contentType: file.contentType,
        uploadUrl: file.uploadUrl
      }))
    });
  } catch (error) {
    if (error instanceof Error && [
      "Choose files or provide a link.",
      "Choose files or provide a link, not both.",
      "Enter a valid URL.",
      "Every file needs a name.",
      "Upload at most 100 files at a time."
    ].includes(error.message)) {
      res.status(400).json({ error: error.message });
      return;
    }

    next(error);
  }
});

appRouter.post("/api/uploads/:uploadId/uploaded", async (req, res, next) => {
  try {
    const user = await getApiUser(req, res);
    if (!user) {
      return;
    }

    const existingUpload = await uploadStore.getUserUpload(user.id, req.params.uploadId);
    const shouldNotify = Boolean(existingUpload && existingUpload.files.length > 0 && !existingUpload.filesUploadedAt);

    await uploadStore.markFilesUploaded(user.id, req.params.uploadId);

    if (shouldNotify) {
      const upload = await uploadStore.getUserUpload(user.id, req.params.uploadId);
      await notifyUploadSubmitted(user, upload ?? existingUpload!);
    }

    res.json({ ok: true });
  } catch (error) {
    next(error);
  }
});

appRouter.post("/uploads/:uploadId/prompt", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const updated = await uploadStore.updatePendingPrompt(
      user.id,
      req.params.uploadId,
      parsePromptInput(req.body)
    );

    if (!updated) {
      res
        .status(409)
        .type("html")
        .send(messagePage("Prompt locked", "Prompts can only be edited while an upload is pending.", "/", "Back to app"));
      return;
    }

    res.redirect(303, "/#previous-uploads");
  } catch (error) {
    next(error);
  }
});

appRouter.get("/uploads/:uploadId/results", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const upload = await uploadStore.getUserUpload(user.id, req.params.uploadId);
    if (!upload || upload.status !== "completed" || !upload.resultObjectName) {
      res
        .status(404)
        .type("html")
        .send(messagePage("Results pending", "Results are not ready for this upload yet.", "/", "Back to app"));
      return;
    }

    const fileName = (upload.resultFileName ?? "shotwell-results").replace(/[\\r\\n"]/g, "");
    res.setHeader("Content-Type", upload.resultContentType ?? "application/octet-stream");
    res.setHeader("Content-Disposition", `attachment; filename="${fileName}"`);

    const stream = createResultReadStream(upload.resultObjectName);
    stream.on("error", next);
    stream.pipe(res);
  } catch (error) {
    next(error);
  }
});

appRouter.get("/auth/complete", async (req, res, next) => {
  try {
    const code = typeof req.query.code === "string" ? req.query.code : "";
    const handoff = await authStore.consumeHandoff(code);

    if (!handoff) {
      res
        .status(400)
        .type("html")
        .send(messagePage("Sign-in expired", "Start the sign-in flow again.", loginUrl(), "Sign in"));
      return;
    }

    const session = await authStore.createSession(handoff.userId, "app");
    setSessionCookie(res, "app", session.id);
    res.redirect(303, "/");
  } catch (error) {
    next(error);
  }
});

appRouter.get("/api/me", async (req, res, next) => {
  try {
    const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

    if (!session) {
      res.status(401).json({ error: "unauthorized" });
      return;
    }

    const user = await authStore.getUser(session.userId);
    if (!user) {
      res.status(401).json({ error: "unauthorized" });
      return;
    }

    res.json({
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        avatarUrl: user.avatarUrl
      }
    });
  } catch (error) {
    next(error);
  }
});
