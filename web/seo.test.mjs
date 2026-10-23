import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

const html = fs.readFileSync(new URL('./index.html', import.meta.url), 'utf8');
const attrs = tag => Object.fromEntries([...tag.matchAll(/([\w:-]+)="([^"]*)"/g)].map(m => [m[1], m[2]]));
const meta = Object.fromEntries([...html.matchAll(/<meta\s[^>]+>/g)].map(m => attrs(m[0])).map(a => [a.name || a.property, a.content]));
const canonical = attrs(html.match(/<link\s[^>]*rel="canonical"[^>]*>/)[0]).href;
const graph = JSON.parse(html.match(/<script type="application\/ld\+json">([\s\S]*?)<\/script>/)[1]);

test('search and share metadata identify one canonical public page and a real image', () => {
  assert.equal(canonical, 'https://c5vrx.com/');
  assert.equal((html.match(/rel="canonical"/g) || []).length, 1);
  assert.equal((html.match(/<title>/g) || []).length, 1);
  assert.ok(meta.description.length > 60 && meta.description.length < 180);
  assert.equal(meta['og:url'], canonical);
  assert.equal(meta['og:description'], meta.description);
  assert.equal(meta['twitter:description'], meta.description);
  assert.equal(meta['og:image'], meta['twitter:image']);
  assert.ok(meta['og:image:alt']);
  assert.equal(new URL(meta['og:image']).origin, new URL(canonical).origin);
  assert.ok(fs.existsSync(new URL(`.${new URL(meta['og:image']).pathname}`, import.meta.url)));
  assert.doesNotMatch(meta.robots, /noindex|nofollow/);
  assert.equal(meta['twitter:card'], 'summary_large_image');
});

test('structured data has resolved internal identities and truthful software provenance', () => {
  assert.equal(graph['@context'], 'https://schema.org');
  const ids = new Set(graph['@graph'].map(item => item['@id']));
  assert.equal(ids.size, graph['@graph'].length);
  const walk = value => {
    if (!value || typeof value !== 'object') return;
    if (value['@id']) assert.ok(ids.has(value['@id']), `Missing graph identity ${value['@id']}`);
    for (const child of Object.values(value)) walk(child);
  };
  walk(graph);
  const code = graph['@graph'].find(item => item['@type'] === 'SoftwareSourceCode');
  assert.equal(code.codeRepository, 'https://github.com/Twotoz/C5VRX');
  assert.ok(code.license.endsWith('/LICENSE'));
  assert.equal(graph['@graph'].filter(item => item['@type'] === 'WebSite').length, 1);
  assert.ok(graph['@graph'].some(item => item['@type'] === 'WebPage' && item.url === canonical));
  assert.ok(!graph['@graph'].some(item => item.aggregateRating || item.offers));
});

test('static content, internal anchors and sitemap remain crawlable without running the app', () => {
  assert.equal((html.match(/<h1(?:\s|>)/g) || []).length, 1);
  const ids = new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]));
  for (const link of html.matchAll(/href="#([^"]+)"/g)) assert.ok(ids.has(link[1]), `Broken anchor #${link[1]}`);
  for (const id of ['about', 'faq']) {
    const section = html.match(new RegExp(`<section[^>]*id="${id}"[^>]*>`));
    assert.ok(section);
    assert.doesNotMatch(section[0], /\bhidden\b/);
  }
  const robots = fs.readFileSync(new URL('./robots.txt', import.meta.url), 'utf8');
  assert.match(robots, /Sitemap: https:\/\/c5vrx\.com\/sitemap\.xml/);
  assert.doesNotMatch(robots, /^Disallow:\s*\/\s*$/m);
  const sitemap = fs.readFileSync(new URL('./sitemap.xml', import.meta.url), 'utf8');
  assert.deepEqual([...sitemap.matchAll(/<loc>([^<]+)<\/loc>/g)].map(m => m[1]), [canonical]);
  assert.ok(sitemap.includes(`<image:loc>${meta['og:image']}</image:loc>`));
  assert.ok(!html.includes('id="alphaBuildWarning"'));
});
